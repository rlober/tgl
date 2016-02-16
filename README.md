| Status (Linux/OSX) | Chat |
|:------------------:|:----:|
|[![Build Status](https://travis-ci.org/syroco/tgl.svg?branch=master)](https://travis-ci.org/syroco/tgl)| [![Join the chat at https://gitter.im/syroco/tgl](https://badges.gitter.im/syroco/tgl.svg)](https://gitter.im/syroco/tgl?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)|

#TGL - Trajectory Generation Library

A set of badass trajectory generation tools.

##Build & Install
###Dependencies
Required:
 - Eigen >3.0
 - Glog

Optional:
 - Doxygen

####Linux
```
sudo apt-get install libeigen3-dev libgoogle-glog-dev doxygen
```
####OSX
blah
####Windows
blah


###Compile & Install
####Linux
```
git clone https://github.com/syroco/tgl.git
cd tgl
mkdir build
cd build
ccmake ..
```
To change the installation directory use `ccmake .. -DCMAKE_INSTALL_PREFIX=[path]`.

For developers enable the tests (ON by default):
```cmake
COMPILE_TESTS                    ON
```
and to build the documentation (OFF by default):
```cmake
GENERATE_DOCUMENTATION           ON
```
To compile and install:
```
sudo make install
```
####OSX
blah
####Windows
blah
