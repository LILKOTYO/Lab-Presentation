# 20230323_Collision_Dtection

## Overview

Code framework and the demo of the presentation.

## Installation

Install **Git** and build system **Cmake**. For windows user, MSVC need to be installed.

### Note for linux users

Many linux distributions do not include `gcc` and the basic development tools in their default installation. On Ubuntu, you need to install the following packages:

```
sudo apt-get install build-essential libx11-dev mesa-common-dev libgl1-mesa-dev libglu1-mesa-dev libxrandr-dev libxi-dev libxmu-dev libblas-dev libxinerama-dev libxcursor-dev
```

### Build

Clone this repo:
```
git clone --recursive https://github.com/LILKOTYO/Lab-Presentation.git
``` 

Run the following commands inside the relevant subfolder to setup the build folder:
```
mkdir build
cd build
cmake ..
```

Compile and run the executable, e.g. Ubuntu:
```
make && ./src/collision
```

### For Windows

Cmake will generate Microsft visual studio project. Click the `*.sln` file in your `[buildFolder]` to open the project and build.
```
mkdir build
cd build
cmake ..

# then find and open the `.sln `file
```
## FAQ

* Command `cmake ../` stuck at `Cloning into 'eigen'...` or other cloning.

    This may caused by problem of accessing github. Please first try use [SJTU VPN](!https://net.sjtu.edu.cn/wlfw/VPN.htm) (or other tools &#x1F92B;) and compile. 

    ~~Also, some repos are available on our own server, such as *Eigen*. The download info of libigl dependencies is located at file `libigl/cmake/LibiglDownloadExternal.cmake`. For example, you can change the link at line 70 to `http://dalab.se.sjtu.edu.cn/gitlab/xiaoyuwei/eigen.git` use eigen mirror repo at DALAB server.~~

    ~~glfw (http://dalab.se.sjtu.edu.cn/gitlab/xiaoyuwei/glfw.git) and imgui (http://dalab.se.sjtu.edu.cn/gitlab/xiaoyuwei/imgui.git) are also available on our server.~~

* The menu bar isn't rendered correctly in Release mode using VS2019.

    Solution 1: Use Debug mode.

    Solution 2: Remove `ImGuiWindowFlags_AlwaysAutoResize` flag used in `ImGui::Begin()` and comment out `ImGui::SetNextWindowSizeConstraints()` in `include/Gui.cpp`. See [#1669](https://github.com/libigl/libigl/issues/1669) for more details.

    

----

The main part of this framework refers to physical-based simulation course at ETH Zurich.
