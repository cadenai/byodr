#!/usr/bin/python3
import datetime
import glob
import json
import logging
import os
import subprocess

logger = logging.getLogger(__name__)

_supported_tegra_releases = ('32.2.0', '32.2.1', '32.3.1')

# The constants are the same as those used in the application docker builds.
APP_USER_ID = 1990
APP_GROUP_ID = 1990
APP_USER_NAME = 'byodr'
APP_GROUP_NAME = 'byodr'
DEV_GROUP_NAME = 'plugdev'

_udev_arduino_rules_file = '/etc/udev/rules.d/99-arduino.rules'
_udev_arduino_rules_contents = '''
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", GROUP="{group}", MODE="0660", SYMLINK+="arduino", TAG+="systemd"
'''.format(**{'idVendor': '{idVendor}', 'idProduct': '{idProduct}', 'group': DEV_GROUP_NAME})

_docker_image_prefix = 'centipede2donald/byodr-ce:'

# Keeping the service file prefix constant enables removing the services in the future even when the tagnames change.
_systemd_service_file_prefix = 'byodr-ce-'
_systemd_system_directory = '/etc/systemd/system'
_systemd_service_container_prefix = 'byodr_ce_'
_systemd_service_description_prefix = 'Byodr CE '

_sd_app_main_service_tag = 'teleop'
_sd_app_main_service_name = _systemd_service_container_prefix + _sd_app_main_service_tag
_sd_app_service_name = _sd_app_main_service_name + '.service'

_docker_component_tags = [
    (_sd_app_main_service_tag, 'Requires=docker.service', 'After=docker.service', 'runc', None, '9100:9100', None, None),
    ('rosserial', 'Wants=dev-arduino.device', 'After=dev-arduino.device', 'runc', None, '11311:11311', '/dev/arduino', None),
    ('pilot', 'PartOf=' + _sd_app_service_name, 'After=' + _sd_app_service_name, 'runc', _sd_app_main_service_name, None, None, None),
    ('rover', 'PartOf=' + _sd_app_service_name,
     'After=' + _systemd_service_container_prefix + 'rosserial.service',
     'runc',
     _sd_app_main_service_name,
     None,
     None,
     'ROS_MASTER_URI=http://{}rosserial:11311'.format(_systemd_service_container_prefix)),
    ('recorder', 'PartOf=' + _sd_app_service_name, 'After=' + _sd_app_service_name, 'runc', _sd_app_main_service_name, None, None, None),
    ('inference', 'PartOf=' + _sd_app_service_name, 'After=' + _sd_app_service_name, 'nvidia', _sd_app_main_service_name, None, None, None)
]

_systemd_service_template = '''
[Unit]
Description={sd_description}
{sd_unit_line0}
{sd_unit_line1}

[Service]
User={sd_service_user}
Group={sd_service_group}
{sd_environment}
TimeoutStartSec=0
Restart=on-failure
ExecStart=/usr/bin/docker run --rm \
    --name {sd_container_name} \
    --runtime {sd_runtime_name} \
    {sd_volumes_from}  \
    {sd_volumes_list} \
    {sd_ports_list} \
    {sd_devices_list} \
    {sd_image_name}
ExecStop=/usr/bin/docker stop {sd_container_name}

[Install]
WantedBy=multi-user.target
'''


def _run(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True):
    return subprocess.run(args, stdout=stdout, stderr=stderr, universal_newlines=universal_newlines)


class StateManager(object):
    _f_tegra_release = '/etc/nv_tegra_release'
    _f_cuda_version = '/usr/local/cuda/version.txt'

    def __init__(self, build_dirname='build', log_prefix='installOrUpdate', default_application_dirname='byodr-ce'):
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
        _fname = StateManager._f_tegra_release
        if os.path.exists(_fname):
            _release, _revision = [x.strip() for x in _run(['head', '-n', '1', _fname]).stdout.split(',')[:2]]
            _tegra_release = '{}.{}'.format(_release.replace('# R', '').replace(' (release)', ''), _revision.replace('REVISION: ', ''))
        self.state['tegra.release'] = _tegra_release
        # Determine the CUDA version.
        _fname = StateManager._f_cuda_version
        self.state['cuda.version'] = _run(['cat', _fname]).stdout.strip().split(' ')[-1] if os.path.exists(_fname) else ''
        _file = os.path.join(self.build_dirname, self.json_filename)
        # Read previous state if any.
        if os.path.exists(_file):
            with open(_file, mode='r') as workfile:
                try:
                    self.state.update(json.load(workfile))
                except Exception as e:
                    logger.warning("Exception reading json file: " + str(e) + "\n")
        # Init application directory.
        if 'application.directory' not in self.state.keys():
            self.state['application.directory'] = os.path.join(self.state['user.home'], self.default_application_dirname)

    def get_application_directory(self):
        return self.state['application.directory']

    def get_config_directory(self):
        return os.path.join(self.get_application_directory(), 'etc')

    def get_sessions_directory(self):
        return os.path.join(self.get_application_directory(), 'sessions')

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


def do_build_directory(user, group, build_dirname):
    if os.path.exists(build_dirname):
        return "The build directory already exists."
    _umask = os.umask(000)
    os.mkdir('build', mode=0o775)
    os.umask(_umask)
    _run(['chown', '{}:{}'.format(user, group), build_dirname])
    _run(['chmod', 'g+s', 'build'])
    return "The build directory was created."


def do_udev_rules():
    if os.path.exists(_udev_arduino_rules_file):
        return "File {} exists.".format(_udev_arduino_rules_file)
    with open(_udev_arduino_rules_file, mode='w') as f:
        f.write(_udev_arduino_rules_contents)
        _run(['/etc/init.d/udev', 'reload'])
        return "Created {}.".format(_udev_arduino_rules_file)


def do_application_user(user):
    _passwd_line = _run(['grep', APP_USER_NAME, '/etc/passwd']).stdout.strip()
    if '{}:'.format(APP_USER_NAME) in _passwd_line:
        return _passwd_line
    _run(['groupadd', '--gid', str(APP_GROUP_ID), '--system', APP_GROUP_NAME])
    _run(['useradd', '--uid', str(APP_USER_ID), '--gid', str(APP_GROUP_ID), '--system', APP_USER_NAME])
    # Setup group membership used for access to the arduino serial device.
    _run(['usermod', '-aG', DEV_GROUP_NAME, APP_USER_NAME])
    # Make sure the regular user has group access as well.
    _run(['usermod', '-aG', APP_GROUP_NAME, user])
    return "Created user {}:{} and group {}:{}.".format(APP_USER_NAME, APP_USER_ID, APP_GROUP_NAME, APP_GROUP_ID)


def do_application_directory(user, application_dir, config_dir, sessions_dir):
    _umask = os.umask(000)
    os.makedirs(application_dir, mode=0o775)
    os.makedirs(config_dir, mode=0o775)
    os.makedirs(sessions_dir, mode=0o775)
    os.umask(_umask)
    _run(['chmod', '-R', 'g+s', application_dir])
    _run(['chown', '-R', '{}:{}'.format(user, APP_GROUP_NAME), application_dir])
    return "The application directory is {}.".format(application_dir)


def do_docker_images(_answer):
    if _answer == 'y':
        for tag_name in [t[0] for t in _docker_component_tags]:
            print("Pulling {}{}".format(_docker_image_prefix, tag_name))
            _run(['docker', 'pull', _docker_image_prefix + tag_name])
        return _run(['docker', 'images']).stdout
    else:
        return "Skipped docker images pull."


def list_service_files():
    return glob.glob(os.path.join(_systemd_system_directory, _systemd_service_file_prefix + '*.service'))


def stop_and_remove_services():
    # Stop the services and remove any existing service definitions.
    file_list = list_service_files()
    for f_path in file_list:
        f_name = os.path.basename(f_path)
        _run(['systemctl', 'stop', f_name])
        _run(['systemctl', 'disable', f_name])
        _run(['rm', f_path])
    return "Stopped and removed services {}.".format(file_list)


def create_services(user, group, config_dir, sessions_dir):
    for component in _docker_component_tags:
        name, line0, line1, runtime, volume_from, ports, device, env = component
        _devices = None if device is None else [device]
        _volumes = ['{}:/config'.format(config_dir), '{}:/sessions'.format(sessions_dir)]
        if runtime == 'nvidia':
            _volumes = _volumes + ['/usr/local/cuda:/usr/local/cuda', '/usr/lib/aarch64-linux-gnu:/usr-extra']
        _file = os.path.join(_systemd_system_directory, _systemd_service_file_prefix + name + '.service')
        with open(_file, mode='w') as f:
            _m = {
                'sd_description': _systemd_service_description_prefix + name,
                'sd_unit_line0': line0,
                'sd_unit_line1': line1,
                'sd_service_user': user,
                'sd_service_group': group,
                'sd_runtime_name': runtime,
                'sd_environment': '' if env is None else 'Environment={}'.format(env),
                'sd_volumes_from': '' if volume_from is None else '--volumes-from {}:rw'.format(volume_from),
                'sd_container_name': _systemd_service_container_prefix + name,
                'sd_volumes_list': ' '.join(['-v {}'] * len(_volumes)).format(*_volumes),
                'sd_ports_list': '' if ports is None else '-p {}'.format(ports),
                'sd_devices_list': '' if _devices is None else ' '.join(['--device {}'] * len(_devices)).format(*_devices),
                'sd_image_name': _docker_image_prefix + name
            }
            f.write(_systemd_service_template.format(**_m))

    _run(['systemctl', 'daemon-reload'])
    file_list = list_service_files()
    for f_path in file_list:
        f_name = os.path.basename(f_path)
        _run(['systemctl', 'enable', f_name])
    return _run(['systemctl', 'list-unit-files', _systemd_service_file_prefix + '*']).stdout


def start_services():
    file_list = list_service_files()
    for f_path in file_list:
        f_name = os.path.basename(f_path)
        _run(['systemctl', 'start', f_name])
    return _run(['systemctl', 'list-units', _systemd_service_file_prefix + '*']).stdout


def main():
    # This script must be run as root.
    if os.environ['USER'] != 'root':
        print("Please run this script as root, e.g. sudo ./{}".format(os.path.basename(__file__)))
        exit(-1)

    # The StateManager collects information about the system.
    manager = StateManager()
    state = manager.state
    user, group = state['user.name'], state['user.group']
    print("Checking build directory.")
    _result = do_build_directory(user, group, manager.build_dirname)
    print(_result)

    with manager:
        manager.log("\n--- {} ---".format(datetime.datetime.today().strftime("%a %b %d %H:%M:%S %Y")))
        manager.log(str(state))
        print("Welcome to the BYODR installer, the following information will be used:")
        print("{} \t\t{}".format('user', state['user.name']))
        print("{} \t\t{}".format('group', state['user.group']))
        print("{} \t\t{}".format('tegra', state['tegra.release']))
        print("{} \t\t{}".format('cuda', state['cuda.version']))

        # Check platform support.
        print("\nChecking platform support.")
        if state['tegra.release'] not in _supported_tegra_releases:
            print("This tegra release is not part of the supported platforms ({}), continue anyway?".format(_supported_tegra_releases))
            print("Type y or no then <ENTER>")
            print("> ", end='')
            _answer = input()
            if _answer != 'y':
                exit(-1)

        print("Ok.")
        print("\nChecking udev rules.")
        _result = do_udev_rules()
        manager.log(_result)
        print(_result)

        # Proceed to setup the application user and group.
        print("\nChecking application user and group.")
        _result = do_application_user(user)
        manager.log(_result)
        print(_result)

        # The application requires a working directory.
        # Read previous work if any.
        print("\nChecking application directory.")
        _app_dir = manager.get_application_directory()
        _config_dir = manager.get_config_directory()
        _sessions_dir = manager.get_sessions_directory()
        if os.path.exists(_app_dir):
            print("Ok.")
        else:
            print("Which directory do you want to use to store data in?")
            print("Type the full path or <ENTER> to use {} or ^C to quit.".format(_app_dir))
            print("> ", end='')
            _answer = input()
            if _answer.startswith('/'):
                _app_dir = _answer
                manager.set_application_directory(_app_dir)
                _config_dir = manager.get_config_directory()
                _sessions_dir = manager.get_sessions_directory()
            _result = do_application_directory(user, _app_dir, _config_dir, _sessions_dir)
            manager.log(_result)
            print(_result)

        # Proceed with the docker images.
        print("\nThis step requires downloading large binary files which takes time and bandwidth, do you want to do this now?")
        print("Type y or no then <ENTER>")
        print("> ", end='')
        _answer = input()
        _result = do_docker_images(_answer)
        manager.log(_result)
        print(_result)

        # Proceed with the systemd services.
        print("\nRemoving systemd services.")
        _result = stop_and_remove_services()
        manager.log(_result)
        print(_result)

        print("\nRecreating systemd services.")
        _result = create_services(user, group, _config_dir, _sessions_dir)
        manager.log(_result)
        print(_result)

        print("\nDo you want to start the services now? The arduino must be connected for this to work.")
        print("Otherwise reboot after having connected the arduino.")
        print("Type y or no then <ENTER>")
        print("> ", end='')
        _answer = input()
        if _answer == 'y':
            _result = start_services()
            manager.log(_result)
            print(_result)

    # Done - script end.
    print("")
    print("The installer finished and a log file can be found in the {} directory.".format(manager.build_dirname))
    print("")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("")
