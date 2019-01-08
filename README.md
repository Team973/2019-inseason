# greybots-skeleton

This is a skeleton for FRC C++ projects using Gradle.

## Table of Contents

-   [Environment Setup/Installation](#environment-setup)
-   [Compiling with Gradle](#compiling-with-gradle)

## Environment Setup

Our current Gradle setup only provides support for macOS, Ubuntu, and other Unix style OS', Windows is not supported. As of now only Ubuntu and macOS are tested.

### macOS Preinstall

Unfortunately, Python 3 is not preinstalled in macOS (unlike Python 2.7). To be able to run the Python 3 install script, please follow the directions on [Homebrew's Website](https://brew.sh) to install Homebrew. After that is installed, run:

    brew install python3

After completion, follow the section below to finish the environment setup.

### Installation

Download the [`install.py`](https://raw.githubusercontent.com/Team973/greybots-skeleton/master/install.py) file (right click, save as).

#### Easy Installation

Run:

    python3 /path/to/install.py install

#### Advanced Usage

    python3 /path/to/install.py <positional argument> <optional arguments>

The following positional arguments are available:

-   `check` - check if dependencies and extras are installed
-   `install` - main installer for environment

The following shared optional arguments are available:

-   `-h, --help` - displays help text explaining how to use the install script.
-   `-q, --quiet` - output only warnings and errors

The following optional arguments are available only for the install argument.

-   `--barebones` - do not build/clone repo, just install
-   `--build` - automatically compile code to test Gradle
-   `--clone-repo` - automatically clone the repo
-   `--github` - clone/build repositories from a specific directory other than ~/Documents/GitHub
-   `--repo` - specify a repo to use for clone/build from Team973
-   `--vscode` - automatically install VSCode IDE
-   `--vscode-packages` - automatically install VSCode packages

## Compiling with Gradle

Build the robot code:

    ./gradlew build

Build and deploy to the robot:

    ./gradlew deploy
