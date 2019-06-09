#!/usr/bin/env python3
'''
Greybots Environment Installer
'''

from argparse import ArgumentParser
from inspect import getdoc
from os import chdir, path as ospath
from pathlib import Path
from shutil import rmtree
from subprocess import Popen, CalledProcessError, PIPE, STDOUT
import sys
from platform import system
from logging import basicConfig, getLogger, INFO, DEBUG, WARNING

LOGGER = getLogger('greybots.installer')

####################
#  Main functions  #
####################

# Main function.


def main(args=None):
    '''
    Main function that runs at the start of the program.

    Arguments:
        args: The arguments passed when the script is called.
    '''

    ##########################
    #  Parser Configuration  #
    ##########################

    # Method handler
    parser = ArgumentParser(description='Greybots Environement Installer')

    # Argument handler
    subparser = parser.add_subparsers(dest='command', help='Commands')
    subparser.required = True

    # Shared arguments
    shared = ArgumentParser(add_help=False)
    shared.add_argument('-v', '--verbose',
                        help='verbose output',
                        action='store_true')
    shared.add_argument('-q', '--quiet',
                        help='output only warnings and errors',
                        action='store_true')

    #####################################
    #  Avaiable Commands Configuration  #
    #####################################

    # List of methods available to the user. Requires _opts method.
    command_methods = [
        'check',
        'install'
    ]

    # Setup avaiable methods
    for command in command_methods:
        cmd_method = getattr(sys.modules[__name__], command)
        method_opt = getattr(sys.modules[__name__], command + '_opts')
        cmd_parser = subparser.add_parser(command,
                                          help=getdoc(cmd_method),
                                          parents=[shared])
        method_opt(cmd_parser)
        cmd_parser.set_defaults(cmdobj=cmd_method, github=Path(
            user_home(), 'Documents', 'GitHub'), repo='2019-inseason')

    # Parse the arguments into an object
    options = parser.parse_args(args)

    #####################
    #  Logger Settings  #
    #####################

    # Logger format
    log_datefmt = '%H:%M:%S'
    log_format = '%(asctime)s:%(msecs)03d %(levelname)-8s: %(name)-20s: %(message)s'

    # Configure Logger
    basicConfig(datefmt=log_datefmt, format=log_format)

    # Logger output threshold customization
    if options.verbose:
        LOGGER.setLevel(DEBUG)
    elif options.quiet:
        LOGGER.setLevel(WARNING)
        sys.tracebacklimit = 0
    else:
        LOGGER.setLevel(INFO)
        sys.tracebacklimit = 0

    ###################
    #  Method Caller  #
    ###################

    # Start the requested script
    LOGGER.info('Greybots Environement Installer')
    options.cmdobj(options)


#########################
#  Installer Functions  #
#########################

# Check command optional arguments.
def check_opts(check_parser):
    '''
    Optional arguments for the check option.

    Arguments:
        check_parser: The check script argument parser.
    '''
    pass


# Run a dependency check.
def check(options):
    '''
    Check if dependencies and extras are installed.

    Arguments:
        options: The options to check with.
    '''

    if system() != 'Windows':
        LOGGER.info('Running dependency checker...')
        LOGGER.debug('Arguments: %s', options)

        LOGGER.info('Git: %s', find_application(['git', '--version']))
        LOGGER.info('clang-format: %s',
                    find_application(['clang-format', '--version']))

        if system() == 'Darwin':
            LOGGER.info('Homebrew: %s', find_application(['brew', '-v']))
            LOGGER.info('JDK 11: %s', find_mac_jdk11())
        else:
            LOGGER.info('Apt: %s', find_application(['apt', '-v']))
            LOGGER.info('JDK 11: %s', find_application(['java', '-version']))
            LOGGER.info('cURL: %s', find_application(['curl', '-V']))

        LOGGER.info('VSCode: %s', find_application(['code', '-v']))

    else:
        raise GeneralError('Platform is not supported.')


# Install command optional arguments.
def install_opts(install_parser):
    '''
    Optional arguments for the install option.

    Arguments:
        install_parser: The install script argument parser.
    '''
    install_parser.add_argument('--barebones',
                                help='do not build/clone repo, just install',
                                action='store_true')
    install_parser.add_argument('--build',
                                help='automatically compile code to test '
                                'Gradle',
                                action='store_true')
    install_parser.add_argument('--clone-repo',
                                help='automatically clone the repo',
                                action='store_true')
    install_parser.add_argument('--github',
                                help='clone/build repositories from a '
                                'specific directory other than '
                                '~/Documents/GitHub')
    install_parser.add_argument('--repo',
                                help='specify a repo to use for clone/build '
                                     'from Team973')
    install_parser.add_argument('--vscode',
                                help='automatically install VSCode IDE',
                                action='store_true')
    install_parser.add_argument('--vscode-packages',
                                help='automatically install VSCode packages',
                                action='store_true')


# Install and configure the environment.
def install(options):
    '''
    Main installer for environment.
    '''
    if system() != 'Windows':

        cache_root = str(Path(user_home(), 'greybots_installer_cache'))
        run(['mkdir', '-p', cache_root])

        LOGGER.debug('Caching files at %s', cache_root)
        LOGGER.info('Running installer...')
        LOGGER.debug('Arguments: %s', options)
        LOGGER.debug('Cache root: %s', cache_root)

        if system() == 'Darwin':

            ######################
            #  Package Managers  #
            ######################

            # Homebrew should already be installed to run python3, but in some
            # rare case, check for it.
            if not find_application(['brew', '-v']):
                raise GeneralError('Please install Homebrew before running.')

            #############################
            #  Development Dependencies #
            #############################

            # JDK 11
            if not find_mac_jdk11():
                LOGGER.info('Installing Java Development Kit...')
                run(['brew', 'tap', 'caskroom/versions'])
                run(['brew', 'cask', 'install', 'java'])

            # clang, macOS has it in xcode-install
            if not find_application(['clang-format', '-version']):
                LOGGER.info('Installing Clang Format...')
                run(['brew', 'install', 'clang-format'])

        elif system() == 'Linux':

            ######################
            #  Package Managers  #
            ######################

            # Apt
            if not find_application(['apt', '-v']):
                raise GeneralError('Apt not installed. Probably not a '
                                   'compatible Debian platform.')
            elif find_application(['apt', '-v']):
                LOGGER.info('Updating and upgrading APT...')
                apt_update()
                apt_upgrade()

            # git
            if not find_application(['git', '--version']):
                LOGGER.info('Installing Git...')
                apt_install('git')

            # curl
            if not find_application(['curl', '-V']):
                LOGGER.info('Installing cURL...')
                apt_install('curl')

            #############################
            #  Development Dependencies #
            #############################

            # JDK 11
            if not find_application(['java', '-version']):
                LOGGER.info('Installing Java Development Kit...')
                apt_install('openjdk-11-jdk')

            # clang-format
            if not find_application(['clang-format', '-version']):
                LOGGER.info('Installing Clang Format...')
                apt_install('clang-format')

        LOGGER.info('Finished installing dependencies')

        ######################
        #  Repository Setup  #
        ######################

        # Ask the user if they want to clone.
        clone_yn = False
        if not (options.clone_repo or options.barebones):
            if query_yes_no('Clone {} repository?'.format(options.repo)):
                LOGGER.debug('User decided to clone')
                clone_yn = True

        # Clone the repository
        if options.clone_repo or clone_yn and not options.barebones:
            github_dir = Path(options.github)
            run(['mkdir', '-p', github_dir])

            repo_url = 'https://github.com/team973/{}'.format(options.repo)
            repo_dir = Path(github_dir, options.repo)

            if not repo_dir.is_dir():
                LOGGER.info('Cloning %s to %s', options.repo, options.github)
                run(['git', 'clone', repo_url, repo_dir])
                LOGGER.warning(
                    'Cloned using HTTPS. SSH needs additional steps.')
            else:
                LOGGER.warning('Repository already exists. Not cloning.')

        ################
        #  Test Build  #
        ################

        # Ask the user if they want to build.
        build_yn = False
        if not (options.build or options.barebones):
            if query_yes_no('Build {} with Gradle?'.format(options.repo)):
                LOGGER.debug('User decided to build')
                build_yn = True

        # Build
        if options.build or build_yn and not options.barebones:
            LOGGER.info('Building using Gradle')

            chdir(Path(options.github, options.repo))

            run(['./gradlew', 'installRoboRioToolchain'])
            run(['./gradlew', 'build'])

        ############
        #  VSCode  #
        ############

        # Ask the user if they want to install VSCode IDE.
        if not find_application(['code', '-v']):
            vscode_yn = False
            if not (options.vscode or options.barebones):
                if query_yes_no('Install VSCode IDE?'):
                    LOGGER.debug('User decided to install VSCode IDE')
                    vscode_yn = True

            # Install VSCode IDE
            if options.vscode or vscode_yn and not options.barebones:
                LOGGER.info('Installing VSCode IDE...')

                if system() == 'Darwin':
                    run(['brew', 'tap', 'caskroom/cask'])
                    run(['brew', 'cask', 'install', 'visual-studio-code'])

                elif system() == 'Linux':
                    run(['curl', '-Lo', Path(cache_root, 'vscode.deb'),
                         'https://go.microsoft.com/fwlink/?LinkID=760868'])
                    apt_install(Path(cache_root, 'vscode.deb'))

                else:
                    raise GeneralError('Platform is not supported')

        # Ask the user if they want to install VSCode pacakges.
        if find_application(['code', '-v']):
            vscode_packages_yn = False
            if not (options.vscode_packages or options.barebones):
                if query_yes_no('Install VSCode packages?'):
                    LOGGER.debug('User decided to install VSCode packages')
                    vscode_packages_yn = True

            # Install VSCode packages
            if options.vscode_packages or vscode_packages_yn and not options.barebones:
                LOGGER.info('Installing VSCode packages...')

                run(['code', '--install-extension', 'bbenoist.Doxygen', '&&', 'code', '--install-extension', 'dbaeumer.vscode-eslint', '&&', 'code', '--install-extension', 'docsmsft.docs-markdown', '&&', 'code', '--install-extension', 'jsayol.firebase-explorer', '&&', 'code', '--install-extension', 'mathiasfrohlich.Kotlin', '&&', 'code', '--install-extension', 'ms-python.python', '&&', 'code', '--install-extension', 'ms-vscode.atom-keybindings', '&&', 'code', '--install-extension', 'ms-vscode.cpptools', '&&', 'code', '--install-extension', 'ms-vscode.github-issues-prs', '&&', 'code', '--install-extension',
                     'PeterJausovec.vscode-docker', '&&', 'code', '--install-extension', 'redhat.java', '&&', 'code', '--install-extension', 'vscjava.vscode-java-debug', '&&', 'code', '--install-extension', 'vscjava.vscode-java-dependency', '&&', 'code', '--install-extension', 'vscjava.vscode-java-pack', '&&', 'code', '--install-extension', 'vscjava.vscode-java-test', '&&', 'code', '--install-extension', 'vscjava.vscode-maven', '&&', 'code', '--install-extension', 'wpilibsuite.vscode-wpilib', '&&', 'code', '--install-extension', 'xaver.clang-format'])

        #############
        #  Cleanup  #
        #############

        LOGGER.info('Cleaning up...')
        rmtree(cache_root)

    else:
        raise GeneralError('Platform is not supported')


####################
#  Shell Commands  #
####################

# Run a shell command.
def run(command):
    '''
    A subprocess.Popen wrapper with logging.

    Arguments:
        command: The shell command to run.
    '''
    LOGGER.debug('=> %s', command)
    try:
        process = Popen(command, stdout=PIPE, stderr=STDOUT)
        exitcode = process.wait()

        with process.stdout:
            for stdout in iter(process.stdout.readline, b''):
                LOGGER.debug('%s: %r', command, stdout)

        if exitcode != 0:
            raise CalledProcessError(exitcode, command)

    except CalledProcessError as exception:
        raise ShellError('{} Failed: {}'.format(command, exception))
    else:
        LOGGER.debug('%s finished successfully', command)


# Search if a particular shell application is installed.
def find_application(name):
    '''
    Application finder, used to check for preinstalled dependencies.

    Arguments:
        name: The name of the application.

    Returns:
        The boolean of wheter the application exists.
    '''
    LOGGER.debug('Searching for %s...', name)

    try:
        run(name)
    except OSError:
        LOGGER.debug('Could not find %s', name)
        return False
    else:
        LOGGER.debug('Found %s', name)
        return True


# Search if java is installed on the mac platform.
# TODO: find the actual exception type
def find_mac_jdk11():
    '''
    Special finder for JDK 11 on Mac... Mac has a fake library built in.

    Returns:
        The boolean of whether JDK 11 exists
    '''
    LOGGER.debug('Searching for macOS JDK 11...')
    try:
        run([Path('/usr/libexec/java_home'), '-v', '11.0.1'])
    except:
        LOGGER.debug('Could not find macOS JDK 11')
        return False
    else:
        LOGGER.debug('Found macOS JDK 11')
        return True


# Apt package installer.
def apt_install(packages):
    '''
    Package installer for Debian-based ems with apt.

    Arguments:
        packages: The apt packages to install.
    '''
    LOGGER.debug('Installing %s using apt', packages)

    run(['sudo', 'apt-get', 'install', '-y', packages])


# Apt source updater.
def apt_update():
    '''
    Update apt's source list.
    '''
    LOGGER.debug('Updating apt')

    run(['sudo', 'apt-get', 'update'])


# Apt upgrader.
def apt_upgrade():
    '''
    Upgrade apt packages.
    '''
    LOGGER.debug('Upgrading apt packages')

    run(['sudo', 'apt-get', 'upgrade', '-y'])


####################
#  Misc Functions  #
####################

# User home directory
def user_home():
    '''
    User home directory returner.
    Some python versions do not support Path.home()
    '''
    try:
        return Path.home()
    except AttributeError:
        return ospath.expanduser('~')


# Yes no prompt
def query_yes_no(question, default='yes'):
    '''
    Ask a yes/no question via input() and return their answer.

    Arguments:
        question: The string that is presented to the user.
        default: The answer if the user just hits the enter key.

    Returns:
        The answer as a boolean.
    '''
    valid = {'yes': True, 'y': True, 'ye': True,
             'no': False, 'n': False}
    if default is None:
        prompt = ' [y/n] '
    elif default == 'yes':
        prompt = ' [Y/n] '
    elif default == 'no':
        prompt = ' [y/N] '
    else:
        raise GeneralError('Invalid default answer: {}'.format(default))

    while True:
        LOGGER.warning(question + prompt)
        try:
            choice = input().lower()

            if default is not None and choice == '':
                return valid[default]
            elif choice in valid:
                return valid[choice]
            else:
                LOGGER.error("Invalid option. Please respond with 'yes' or "
                             " 'no' (or 'y' or 'n')")
        except ValueError:
            LOGGER.error("An error occured. Try again.")


###################
#  Error Classes  #
###################

# A general error, typically used for unexpected failures.
class GeneralError(Exception):
    '''
    General error handler.
    '''

    def __init__(self, message, *args):
        super(GeneralError, self).__init__(message)
        self.message = message
        LOGGER.exception('GeneralError: %s', self.message, *args)
        exit(1)


# General shell command error handler.
class ShellError(Exception):
    '''
    A shell error. Used when a shell command doesn't exist or exits non-zero.
    '''

    def __init__(self, message, *args):
        super(ShellError, self).__init__(message)
        self.message = message
        LOGGER.exception('ShellError: %s', self.message, *args)
        exit(2)


# Run the main function at start.
if __name__ == '__main__':
    main()
