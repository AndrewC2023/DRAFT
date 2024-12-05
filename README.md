# Planning Algorithm Development

A collection of MATLAB, Python, & C++ Planning/navigation algorithms and their tracked development

## Development Environment Setup

It is recommended to clone and develop the repository in ubuntu 24.04

### C++ Setup

The required dependencies for the C++ algorithm library are:

* Eigen 3
* Yaml-cpp
* Matplot++
* boost
* pcl

These dependencies must be installed before building the system, you may use the installRequiredLibs script in C++/admin to do this for you. From home directory:

```console
cd C++/admin
sudo ./installRequiredLibs.sh
```

Once dependecies are installed you can run setup.sh to make your build directories and configure them. this may also require you to allow privileges.

```console
sudo ./setup.sh
```

Will pivot to Dev container soon
