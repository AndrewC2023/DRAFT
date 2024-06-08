# Planning Algorithm Development
A collection of MATLAB, Python, & C++ Planning/navigation algorithms and their tracked development

# Development Environment Setup

It is recommended to clone and develop the repository in ubuntu (20.04 or later) as the setup scripts are in shell

To clone the repository:
```console
# ssh
git clone git@github.com:AndrewC2023/PlanningAlgorithmDevelopment.git

# http
git clone https://github.com/AndrewC2023/PlanningAlgorithmDevelopment.git
```

### C++ Setup

The required dependencies for the C++ algorithm library are:
* Eigen 3
* Yaml-cpp
* Matplot++

These dependencies must be installed before building the system, you may use the installRequiredLibs shell script in C++/admin to do this for you. From home directory:

```console
cd C++/admin
sudo ./installRequiredLibs.sh
```

you may need to give the shell script permissions befor it is run with:

```console
chmod +x installRequiredLibs.sh
```

specify where you want matplot++ to be installed. It will take some time to build and install matplot++.

if you elect to install libraries within the project directory you may want to exclude those directories from project searches, in VS Code this can be done in the .vscode/setting.json file by adding:

```json
"search.exclude": {
    "**/C++/admin/reqLib/**": true,
}
```

Once dependecies are installed you can run setup.sh to make your build directories and configure them. this may also require you to allow privileges with chmod.

```console
sudo ./setup.sh
```

### Python Setup
Make sure you have python installed. in windows you can easily go to the website and instal python and add it to your PATH variable. In ubuntu you can get python using your package manager:

```console
sudo apt update
sudo apt install python3
```

this will also instal pip which with manage your python libraries

there is a required libraries shell script in the python folder, but the process of manually using pip in the terminal is not difficult so either approach is fine.

### MATLAB Setup
Make sure you have matlab lnstalled with your desired toolboxes
