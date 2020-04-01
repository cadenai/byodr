#!/usr/bin/python3
import datetime
import json
import os
import subprocess

_f_tegra_release = '/etc/nv_tegra_release'
_f_cuda_version = '/usr/local/cuda/version.txt'

_supported_tegra_releases = ('28.2.1', '32.3.1')

_platform = dict()

# The constants are the same as those used in the application docker builds.
_app_user_id = 1990
_app_group_id = 1990
_app_user_name = 'byodr'
_app_group_name = 'byodr'

_docker_image_prefix = 'centipede2donald/byodr-ce:'
_docker_application_tags = ('rosserial',
                            'teleop',
                            'pilot',
                            'rover',
                            'recorder',
                            'inference')


def _run(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True):
    return subprocess.run(args, stdout=stdout, stderr=stderr, universal_newlines=universal_newlines)


def determine_platform_state():
    # The terminal user regardless of nested sudo.
    _user = _run(['who', 'am', 'i']).stdout.split(' ')[0]
    _platform['user.name'] = _user
    _platform['user.group'] = _run(['id', '-gn', _user]).stdout.strip()
    _platform['user.home'] = os.environ['HOME']
    # Determine the tegra release from disk.
    _tegra_release = ''
    if os.path.exists(_f_tegra_release):
        _release, _revision = [x.strip() for x in _run(['head', '-n', '1', _f_tegra_release]).stdout.split(',')[:2]]
        _tegra_release = '{}.{}'.format(_release.replace('# R', '').replace(' (release)', ''), _revision.replace('REVISION: ', ''))
    _platform['tegra.release'] = _tegra_release
    # Determine the CUDA version.
    _platform['cuda.version'] = _run(['cat', _f_cuda_version]).stdout.strip().split(' ')[-1] if os.path.exists(_f_cuda_version) else ''


def do_build_directory():
    _user = _platform['user.name']
    _group = _platform['user.group']
    if not os.path.exists('build'):
        _umask = os.umask(000)
        os.mkdir('build', mode=0o775)
        os.umask(_umask)
        _run(['chown', '{}:{}'.format(_user, _group), 'build'])
        _run(['chmod', 'g+s', 'build'])


def do_app_user(logfile):
    _user = _platform['user.name']
    _passwd_line = _run(['grep', _app_user_name, '/etc/passwd']).stdout.strip()
    logfile.write(_passwd_line + "\n")
    if '{}:'.format(_app_user_name) not in _passwd_line:
        _run(['groupadd', '--gid', _app_group_id, '--system', _app_group_name])
        _run(['useradd', '--uid', _app_user_id, '--gid', _app_group_id, '--system', _app_user_name])
        # Make sure the regular user has access.
        _run(['usermod', '-aG', _app_group_name, _user])
        logfile.write("Created user {}:{} and group {}:{}.\n".format(
            _app_user_name,
            _app_user_id,
            _app_group_name,
            _app_group_id
        ))


def read_work_state(_f_work_file, logfile):
    _work_state = dict()
    if os.path.exists(_f_work_file):
        with open(_f_work_file, mode='r') as workfile:
            try:
                _work_state = json.load(workfile)
                logfile.write("Installer state contains {}.\n".format(str(_work_state)))
            except Exception as e:
                logfile.write("Exception reading json file: " + str(e) + "\n")
    return _work_state


def do_application_directory(_answer, _app_dir, logfile):
    if _answer.startswith('/'):
        _app_dir = _answer

    _user = _platform['user.name']
    logfile.write("Using application directory {}.\n".format(_app_dir))
    if not os.path.exists(_app_dir):
        _umask = os.umask(000)
        os.makedirs(_app_dir, mode=0o775)
        os.umask(_umask)
    _run(['chown', '{}:{}'.format(_user, _app_group_name), _app_dir])
    _run(['chmod', 'g+s', _app_dir])
    return _app_dir


def do_docker_images(_answer, logfile):
    if _answer == 'y':
        for tag_name in _docker_application_tags:
            print("Pulling {}{}".format(_docker_image_prefix, tag_name))
            _run(['docker', 'pull', _docker_image_prefix + tag_name])
    logfile.write(_run(['docker', 'images']).stdout + "\n")


def main():
    # This script must be run as root.
    if os.environ['USER'] != 'root':
        print("Please run this script as root, e.g. sudo ./{}".format(os.path.basename(__file__)))
        exit(-1)

    # Collect information about the system.
    determine_platform_state()
    print("Welcome to the BYODR installer, the following information will be used:")
    for k in _platform.keys():
        print("{} \t\t{}".format(k, _platform[k]))

    # Check platform support.
    print("Checking platform support.")
    if _platform['tegra.release'] not in _supported_tegra_releases:
        print("This tegra release is not part of the supported platforms ({}), do you want to continue?".format(_supported_tegra_releases))
        print("Type y or no then <ENTER>")
        print("> ", end='')
        _answer = input()
        if _answer != 'y':
            exit(-1)

    # The user may interrupt and restart this script or run it more than once.
    print("Checking build directory.")
    do_build_directory()

    _f_work_file = os.path.join('build', 'installOrUpdate.json')
    _f_log_file = os.path.join('build', 'installOrUpdate.log')

    # Proceed to setup the application user and group.
    _user = _platform['user.name']
    _home = _platform['user.home']
    print("Checking application user and group.")
    with open(_f_log_file, mode='a') as logfile:
        logfile.write("\n--- {} ---\n".format(datetime.datetime.today().strftime("%a %b %d %H:%M:%S %Y")))
        logfile.write(str(_platform) + "\n")

        # Setup the application user and group.
        do_app_user(logfile)

        # The application requires a working directory.
        # Read previous work if any.
        _work_state = read_work_state(_f_work_file, logfile)
        _app_dir = _work_state.get('application.directory', os.path.join(_home, 'byodr-ce'))
        print("Which directory do you want to use to store data in?")
        print("Type the full path or <ENTER> to use {}.".format(_app_dir))
        print("> ", end='')
        _answer = input()
        _app_dir = do_application_directory(_answer, _app_dir, logfile)
        print("The application directory is {}.".format(_app_dir))
        with open(_f_work_file, mode='w') as workfile:
            _work_state['application.directory'] = _app_dir
            json.dump(_work_state, workfile)

        # Proceed with the docker images.
        print("This step requires downloading large binary files which takes time and bandwidth, do you want to do this now?")
        print("Type y or no then <ENTER>")
        print("> ", end='')
        _answer = input()
        do_docker_images(_answer, logfile)

        # Proceed with the systemd services.

    # Done - script end.
    print("The installer finished and a log file can be found at {}.".format(_f_log_file))
    print("")


if __name__ == "__main__":
    main()
