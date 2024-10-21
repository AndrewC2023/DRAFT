# Planning Algorithm Development

A collection of MATLAB, Python, & C++ Planning/navigation algorithms and their tracked development

## Development Environment Setup

It is recommended to clone and develop the repository in ubuntu (20.04 or later)

### C++ Setup

The required dependencies for the C++ algorithm library are:

* Eigen 3
* Yaml-cpp
* Matplot++

These dependencies must be installed before building the system, you may use the installRequiredLibs script in C++/admin to do this for you. From home directory:

```console
cd C++/admin
sudo ./installRequiredLibs.sh
```

you may need to give the shell script permissions before it is run:

```console
chmod +x installRequiredLibs.sh
```

Once dependecies are installed you can run setup.sh to make your build directories and configure them. this may also require you to allow privileges.

```console
sudo ./setup.sh
```
