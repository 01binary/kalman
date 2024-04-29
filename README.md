# Kalman Filter Demo

This is a companion repository for an article about the Kalman filter.

In the article we identify a discrete state-space system model by using System Identification app in Matlab from [input.csv](./input.csv) file.

Sample code in this repository then demonstrates how to use the system model to implement a Kalman filter, first in Matlab and then in C++.

## Matlab

Launch Matlab, set current directory to this repository, then execute `kalman.m` simply by typing `kalman` into Matlab prompt and pressing ENTER.

## C++

The C++ example can be built and executed with the following steps.

## Installation

+ Download [Eigen 3.4](https://gitlab.com/libeigen/eigen/-/releases/3.4.0)
+ Extract and change into the folder: `cd eigen-3.4.0`
+ Create a build folder: `mkdir build && cd build`
+ Setup CMake: `cmake ..`
+ Install: `make install`

## Building

+ Create a build folder: `mkdir build && cd build`
+ Setup CMake: `cmake ..`
+ Build: `make`
+ This should create `kalman` executable in `build` folder

## Running

Run the demo from the current directory:

```
./build/kalman
```

This will read inputs from `input.csv` and write filtered result into `output.csv`.
