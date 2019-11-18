# shm_server
Server for managing connections via shared memory using [`shm_comm`](https://github.com/RCPRG-ros-pkg/shm_comm) library

## Introduction

When writing applications which use [`shm_comm`](https://github.com/RCPRG-ros-pkg/shm_comm) library as mechanism for interprocess communication, you have to deal with shared memory channels' initialization. In the simplest case, you can try to create that channels in every node, but it increases responsibilities of every application. Also, it increases the risk, that when the last "owner" of the channel crash, channel will be not removed and its resources are not freed (because they are interprocess, shared). 

Better approach is to initialize that channels aside, in the other process. Such program have only to create needed channels and remove them at exit, taking care of releasing resources. This process have to be as simple as possible, so the risk of crash is much lower. Finally, our workers-apps have not to worry about channels initialization, how much readers will be, how big buffers must be. They should just use them. It simplifies design.

This project applies second aproach. It gives `shm_server` ROS application, which manages shared memory based communications.

## Building

This project is designed to work with `Catkin`, so build it like any other ROS project. It has only two main dependencies: `shm_comm`, and `roscpp`. In order to build it you need a compiler with C++17 support (at least GCC 7.3.0, by default available in Ubuntu 18.04's APT).

## Using

Every shared-memory server handles only one (at the moment) `shm::Channel` instance. In order to run it, just type:

```sh
roslaunch shm_server shm_server.launch name:=my_channel size:=123 readers:=5
```

where:

- `name` - Name of the shared memory channel, has to be max 128 chars wide
- `size` - Size in bytes of shared memory buffer, has to be greater than zero
- `readers` - Number of readers, has to be greater than zero

Since it works within ROS, you can kill it gracefully with CTRL+C, and it will remove created channel at exit.
