This repository provides MATLAB mex bindings for the library pinocchio (https://github.com/stack-of-tasks/pinocchio). To install pinocchio follow the instructions provided here (https://stack-of-tasks.github.io/pinocchio/download.html). This code has been tested on Ubuntu 20.04. Befor building the code, ensure that you have installed MATLAB on your Linux machine with all the required environment variables set so that CMAKE can find MATLAB headers. The easiest way for that is to install matlab support for Ubuntu:
```
sudo apt-get install matlab-support
sudo apt-get install matlab-support-dev
```
Once everything is set, follow the instructions
```
mkdir build && cd build
cmake ..
make
```
This will generate pin.mex file in the build folder. To test the binding, run the test.m file in the script folder.