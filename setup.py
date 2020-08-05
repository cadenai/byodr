#!/usr/bin/python3
import datetime
import json
import logging
import os
import subprocess

logger = logging.getLogger(__name__)


def _run(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True):
    return subprocess.run(args, stdout=stdout, stderr=stderr, universal_newlines=universal_newlines)


def do_build_directory(user, group, build_dirname):
    if os.path.exists(build_dirname):
        return "The build directory already exists."
    _umask = os.umask(000)
    os.mkdir('build', mode=0o775)
    os.umask(_umask)
    _run(['chown', '{}:{}'.format(user, group), build_dirname])
    _run(['chmod', 'g+s', 'build'])
    return "The build directory was created."


def remove_udev_rules(fname):
    if os.path.exists(fname):
        _run(['rm', fname])
        _run(['/etc/init.d/udev', 'reload'])
    return "Removed {}.".format(fname)


def create_udev_rules(fname, contents):
    if os.path.exists(fname):
        return "File {} exists.".format(fname)
    with open(fname, mode='w') as f:
        f.write(contents)
        _run(['/etc/init.d/udev', 'reload'])
        return "Created {}.".format(fname)


def pull_docker_images(proceed, docker_files, environment, fn_callback):
    if proceed:
        _command = ['docker-compose'] + [y for x in map(lambda df: ['-f', df], docker_files) for y in x] + ['pull']
        process = subprocess.Popen(_command, env=environment, stdout=subprocess.PIPE)
        while True:
            if process.poll() is not None:
                break
            output = process.stdout.readline()
            if output:
                fn_callback(str(output.strip()))
        return _run(['docker', 'images']).stdout
    else:
        return "Skipped docker images pull."


def stop_and_remove_services(service_name, system_directory='/etc/systemd/system'):
    # Stop the services and remove any existing service definitions.
    _run(['systemctl', 'stop', service_name])
    _run(['systemctl', 'disable', service_name])
    _run(['rm', os.path.join(system_directory, service_name)])
    return "Stopped and removed {}.".format(service_name)


def create_services(service_name, service_contents, system_directory='/etc/systemd/system'):
    with open(os.path.join(system_directory, service_name), mode='w') as f:
        f.write(service_contents)
    _run(['systemctl', 'daemon-reload'])
    _run(['systemctl', 'enable', service_name])
    return _run(['systemctl', 'list-unit-files', service_name]).stdout


def start_services(service_name):
    _run(['systemctl', 'start', service_name])
    return _run(['systemctl', 'list-units', service_name]).stdout


class TegraStateManager(object):
    _f_tegra_release = '/etc/nv_tegra_release'
    _f_cuda_version = '/usr/local/cuda/version.txt'

    def __init__(self, build_dirname='build', log_prefix='setup', default_application_dirname='byodrapp'):
        self.build_dirname = build_dirname
        self.log_filename = log_prefix + '.log'
        self.json_filename = log_prefix + '.json'
        self.default_application_dirname = default_application_dirname
        self.state = dict()
        self.open_log = None
        self._init_state()

    def _init_state(self):
        # Find the terminal user regardless of nested sudo.
        _user = _run(['who', 'am', 'i']).stdout.split(' ')[0]
        self.state['user.name'] = _user
        self.state['user.group'] = _run(['id', '-gn', _user]).stdout.strip()
        self.state['user.home'] = os.environ['HOME']
        # Determine the tegra release from disk.
        _tegra_release = ''
        _fname = TegraStateManager._f_tegra_release
        if os.path.exists(_fname):
            _release, _revision = [x.strip() for x in _run(['head', '-n', '1', _fname]).stdout.split(',')[:2]]
            _tegra_release = '{}.{}'.format(_release.replace('# R', '').replace(' (release)', ''), _revision.replace('REVISION: ', ''))
        self.state['tegra.release'] = _tegra_release
        # Determine the CUDA version.
        _fname = TegraStateManager._f_cuda_version
        self.state['cuda.version'] = _run(['cat', _fname]).stdout.strip().split(' ')[-1] if os.path.exists(_fname) else ''
        _file = os.path.join(self.build_dirname, self.json_filename)
        # Read previous state if any.
        if os.path.exists(_file):
            with open(_file, mode='r') as workfile:
                try:
                    self.state.update(json.load(workfile))
                except Exception as e:
                    logger.warning("Exception reading json file: " + str(e) + "\n")
        # Inits.
        if 'application.directory' not in self.state.keys():
            self.state['application.directory'] = os.path.join(self.state['user.home'], self.default_application_dirname)

    def get_application_directory(self):
        return self.state['application.directory']

    def get_sessions_directory(self):
        return os.path.join(self.get_application_directory(), 'sessions')

    def get_config_directory(self):
        return os.path.join(self.get_application_directory(), 'config')

    def get_config_filepath(self):
        return os.path.join(self.get_config_directory(), 'config.ini')

    def set_application_directory(self, dirname):
        self.state['application.directory'] = dirname

    def __enter__(self):
        self.open_log = open(os.path.join(self.build_dirname, self.log_filename), mode='a')
        return self

    def __exit__(self, *args):
        self.open_log.close()
        with open(os.path.join(self.build_dirname, self.json_filename), mode='w') as f:
            json.dump({'application.directory': self.state['application.directory']}, f)

    def log(self, text, new_line=True):
        self.open_log.write(text)
        if new_line:
            self.open_log.write("\n")


class TegraInstaller(object):
    _supported_tegra_releases = ('32.2.0', '32.2.1', '32.3.1')

    # The constants are the same as those used in the application docker builds.
    APP_USER_ID = 1990
    APP_GROUP_ID = 1990
    APP_USER_NAME = 'byodr'
    APP_GROUP_NAME = 'byodr'

    _udev_relay_rules_file = '/etc/udev/rules.d/99-byodr.rules'
    _udev_relay_rules_contents = '''
SUBSYSTEM=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", GROUP="{group}", MODE="0660", SYMLINK+="usbrelay", TAG+="systemd"
'''.format(**{'idVendor': '{idVendor}', 'idProduct': '{idProduct}', 'group': APP_GROUP_NAME})

    _systemd_system_directory = '/etc/systemd/system'
    _systemd_service_name = 'byodr.service'
    _systemd_service_template = '''
[Unit]
Description=Byodr CE Onboard Processes
Requires=docker.service
After=docker.service

[Service]
User={sd_service_user}
Group={sd_service_group}
Environment=DC_CONFIG_DIR={sd_config_dir}
Environment=DC_RECORDER_SESSIONS={sd_sessions_dir}
TimeoutStartSec=0
Restart=on-failure
ExecStart=/usr/bin/docker-compose {sd_compose_files} up 
ExecStop=/usr/bin/docker-compose {sd_compose_files} down -v

[Install]
WantedBy=multi-user.target
'''

    _application_config_template = '''
[camera]
camera.user = User
camera.password = HelloUser
camera.ptz.flip = tilt

[vehicle] 
ras.master.uri = tcp://raspberrypi
ras.throttle.domain.forward.shift = 4
ras.throttle.domain.backward.shift = 0
ras.throttle.domain.scale = 10
ras.throttle.reverse.gear = -25
'''

    def __init__(self, script_location, build_dirname='build'):
        self._docker_files = [
            os.path.join(script_location, 'docker', 'docker-compose.yml'),
            os.path.join(script_location, 'docker', 'docker-compose.tegra.yml')]
        self.manager = TegraStateManager(build_dirname=build_dirname)

    def get_state(self):
        return self.manager.state

    def get_user(self):
        return self.manager.state['user.name']

    def get_group(self):
        return self.manager.state['user.group']

    @staticmethod
    def use_application_directory():
        return True

    def get_application_directory(self):
        return self.manager.get_application_directory()

    def __enter__(self):
        self.manager.__enter__()
        return self

    def __exit__(self, *args):
        self.manager.__exit__(*args)

    def log(self, text, new_line=True):
        self.manager.log(text, new_line)

    def do_application_user(self):
        ti = TegraInstaller
        user = self.get_user()
        _passwd_line = _run(['grep', ti.APP_USER_NAME, '/etc/passwd']).stdout.strip()
        if '{}:'.format(ti.APP_USER_NAME) in _passwd_line:
            return _passwd_line
        _run(['groupadd', '--gid', str(ti.APP_GROUP_ID), '--system', ti.APP_GROUP_NAME])
        _run(['useradd', '--uid', str(ti.APP_USER_ID), '--gid', str(ti.APP_GROUP_ID), '--system', ti.APP_USER_NAME])
        # Make sure the regular user has group access as well.
        _run(['usermod', '-aG', ti.APP_GROUP_NAME, user])
        return "Created user {}:{} and group {}:{}.".format(ti.APP_USER_NAME, ti.APP_USER_ID, ti.APP_GROUP_NAME, ti.APP_GROUP_ID)

    def do_application_directory(self, application_dir):
        ti = TegraInstaller
        user = self.get_user()
        self.manager.set_application_directory(application_dir)
        _umask = os.umask(000)
        os.makedirs(application_dir, mode=0o775, exist_ok=True)
        os.makedirs(self.manager.get_config_directory(), mode=0o775, exist_ok=True)
        os.makedirs(self.manager.get_sessions_directory(), mode=0o775, exist_ok=True)
        os.umask(_umask)
        _run(['chown', '-R', '{}:{}'.format(user, ti.APP_GROUP_NAME), self.manager.get_config_directory()])
        _run(['chown', '-R', '{}:{}'.format(user, ti.APP_GROUP_NAME), self.manager.get_sessions_directory()])
        _run(['chmod', '-R', 'g+s', self.manager.get_sessions_directory()])
        return "The application directory is {}.".format(application_dir)

    def do_application_config_file(self):
        ti = TegraInstaller
        user = self.get_user()
        config_file = self.manager.get_config_filepath()
        if not os.path.exists(config_file):
            with open(config_file, mode='w') as f:
                f.write(ti._application_config_template)
            _run(['chmod', '664', config_file])
            _run(['chown', '{}:{}'.format(user, ti.APP_GROUP_NAME), config_file])
        return _run(['cat', config_file]).stdout

    def pull_docker_images(self, proceed, fn_output):
        return pull_docker_images(proceed, self._docker_files, {'DC_CONFIG_DIR': '', 'DC_RECORDER_SESSIONS': ''}, fn_output)

    @staticmethod
    def stop_and_remove_services():
        result = remove_udev_rules(TegraInstaller._udev_relay_rules_file)
        return result + '\n' + stop_and_remove_services(TegraInstaller._systemd_service_name)

    def create_services(self):
        result = create_udev_rules(TegraInstaller._udev_relay_rules_file, TegraInstaller._udev_relay_rules_contents)
        _m = {
            'sd_service_user': self.get_user(),
            'sd_service_group': self.get_group(),
            'sd_config_dir': self.manager.get_config_directory(),
            'sd_sessions_dir': self.manager.get_sessions_directory(),
            'sd_compose_files': ' '.join('-f {}'.format(name) for name in self._docker_files)
        }
        _contents = TegraInstaller._systemd_service_template.format(**_m)
        return result + '\n' + create_services(TegraInstaller._systemd_service_name, _contents)

    @staticmethod
    def start_services():
        return start_services(TegraInstaller._systemd_service_name)


class PiInstaller(object):
    _systemd_system_directory = '/etc/systemd/system'
    _systemd_service_name = 'byodr.service'
    _systemd_service_template = '''
[Unit]
Description=Byodr CE Onboard Processes
Requires=docker.service
After=docker.service

[Service]
User={sd_service_user}
Group={sd_service_group}
TimeoutStartSec=0
Restart=on-failure
ExecStart=/usr/bin/docker-compose {sd_compose_files} up 
ExecStop=/usr/bin/docker-compose {sd_compose_files} down -v

[Install]
WantedBy=multi-user.target
'''

    def __init__(self, script_location, build_dirname='build', log_prefix='setup'):
        self.build_dirname = build_dirname
        self._docker_files = [os.path.join(script_location, 'raspi', 'docker-compose.yml')]
        self.log_filename = log_prefix + '.log'
        self.state = dict()
        self.open_log = None
        self._init_state()

    def _init_state(self):
        # Find the terminal user regardless of nested sudo.
        _user = _run(['who', 'am', 'i']).stdout.split(' ')[0]
        self.state['user.name'] = _user
        self.state['user.group'] = _run(['id', '-gn', _user]).stdout.strip()
        self.state['user.home'] = os.environ['HOME']

    def get_state(self):
        return self.state

    def get_user(self):
        return self.state['user.name']

    def get_group(self):
        return self.state['user.group']

    @staticmethod
    def use_application_directory():
        return False

    @staticmethod
    def do_application_user():
        return "Ok"

    def __enter__(self):
        self.open_log = open(os.path.join(self.build_dirname, self.log_filename), mode='a')
        return self

    def __exit__(self, *args):
        self.open_log.close()

    def log(self, text, new_line=True):
        self.open_log.write(text)
        if new_line:
            self.open_log.write("\n")

    def pull_docker_images(self, proceed, fn_output):
        return pull_docker_images(proceed, self._docker_files, {'DC_CONFIG_DIR': '', 'DC_RECORDER_SESSIONS': ''}, fn_output)

    @staticmethod
    def stop_and_remove_services():
        return stop_and_remove_services(PiInstaller._systemd_service_name)

    def create_services(self):
        _m = {
            'sd_service_user': self.get_user(),
            'sd_service_group': self.get_group(),
            'sd_compose_files': ' '.join('-f {}'.format(name) for name in self._docker_files)
        }
        _contents = PiInstaller._systemd_service_template.format(**_m)
        return create_services(PiInstaller._systemd_service_name, _contents)

    @staticmethod
    def start_services():
        return start_services(PiInstaller._systemd_service_name)


def create_installer(script_location, build_dirname):
    machine = _run(['uname', '-m']).stdout.strip()
    if machine == 'armv7l':
        return PiInstaller(script_location=script_location, build_dirname=build_dirname)
    else:
        return TegraInstaller(script_location=script_location, build_dirname=build_dirname)


def main():
    # This script must be run as root.
    if os.environ['USER'] != 'root':
        print("Please run this script as root, sudo ./{}".format(os.path.basename(__file__)))
        exit(-1)

    _script_location = os.path.join(os.getcwd(), os.path.dirname(__file__))
    build_dirname = 'build'

    installer = create_installer(script_location=_script_location, build_dirname=build_dirname)
    state, user, group = installer.get_state(), installer.get_user(), installer.get_group()
    print("Checking build directory.")
    _result = do_build_directory(user, group, build_dirname)
    print(_result)

    with installer:
        installer.log("\n--- {} ---".format(datetime.datetime.today().strftime("%a %b %d %H:%M:%S %Y")))
        print("Welcome to the installer, the following information will be used:")
        max_len = max(len(l) for l in state.keys())
        for key in state.keys():
            print('{}{}'.format(key.ljust(max_len + 1), state[key]))

        print("\nChecking application user and group.")
        _result = installer.do_application_user()
        installer.log(_result)
        print(_result)

        if installer.use_application_directory():
            _app_dir = installer.get_application_directory()
            print("\nChecking application directory.")
            print("Which directory do you want to use to store data in?")
            print("Type the full path or <ENTER> to use {} or ^C to quit.".format(_app_dir))
            print("> ", end='')
            _answer = input()
            if _answer.startswith('/') or _answer.startswith('~'):
                _app_dir = os.path.expanduser(_answer)
            else:
                print("\nYour path did not start with '/' or '~', ignored.")
            _result = installer.do_application_directory(_app_dir)
            installer.log(_result)
            print(_result)
            # Make sure there is an application config file.
            print("\nChecking application configuration file.")
            _result = installer.do_application_config_file()
            installer.log(_result)
            print(_result)

        print("\nThis step requires downloading large binary files which takes time and bandwidth, do you want to proceed?")
        print("Type y or no then <ENTER>")
        print("> ", end='')

        def _fn_output(s):
            installer.log(s)
            print(s)

        _answer = input()
        _docker_proceed = _answer == 'y'
        _result = installer.pull_docker_images(_docker_proceed, _fn_output)
        installer.log(_result)
        print(_result)

        print("\nDo you want to (re)create the system services?")
        print("> ", end='')
        _answer = input()
        if _answer == 'y':
            print("\nRemoving systemd services.")
            _result = installer.stop_and_remove_services()
            installer.log(_result)
            print(_result)

            print("\nRecreating systemd services.")
            _result = installer.create_services()
            installer.log(_result)
            print(_result)

            print("\nDo you want to start the services?")
            print("> ", end='')
            _answer = input()
            if _answer == 'y':
                _result = installer.start_services()
                installer.log(_result)
                print(_result)

    print("")
    print("The installer finished and a log file can be found in directory '{}'.".format(build_dirname))
    print("")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("")
