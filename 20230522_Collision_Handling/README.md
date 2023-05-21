# 20230522_Collision_Handling

PPT and its corresponding .pdf file are in the root directory. 

All images and videos in the ppt are placed in  `./pic`.

## Demo
### Interior point method
You can find the demo of the interior point method in my simulator `Ryao`, [link](https://github.com/LILKOTYO/Ryao/tree/master).

You can use [vcpkg](https://github.com/microsoft/vcpkg) to install dependencies (take 64-bit windows system as an example):
```shell
vcpkg install spdlog:x64-windows
vcpkg install eigen3:x64-windows
vcpkg install glad:x64-windows
vcpkg install glfw3:x64-windows
vcpkg install glm:x64-windows
```
Run the code:
```shell
# move to the root dir
cmake -S . -B build -D CMAKE_BUILD_TYPE=Release
cmake --build build
```

### Impact zone optimization
Flex is a particle-based simulation library designed for real-time applications. Check this [link](https://github.com/NVIDIAGameWorks/FleX) to download Flex and run the official demo.
