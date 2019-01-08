### Installation

This install script is only for macOS and debian-based distros. Download the [`install.py`](https://raw.githubusercontent.com/Team973/greybots-skeleton/master/install.py) file (right click, save as).

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
