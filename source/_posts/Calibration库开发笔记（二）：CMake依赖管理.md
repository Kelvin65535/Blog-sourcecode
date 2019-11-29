---
title: Calibration库开发笔记（二）：CMake依赖管理
date: 2019-11-29 15:14:29
tags: C++ Robotic OpenCV
---

本文是Calibration库的第二篇开发笔记，在开发Calibration库的过程中，会陆续将遇到的问题和解决方案记录下来，以供日后查阅和参考。

在此次开发中我打算解决Calibration库依赖的第三方库导入问题。

<!-- more -->

# 项目依赖分析

经过计划我打算先实现棋盘格标定算法的封装，由于在OpenCV中已经有棋盘格标定算法的实现，所以我打算直接使用OpenCV的实现。另外我需要使用文件来持久化相机内参数的标定结果，所以我需要将标定完毕的内参数以yaml的格式导出到本地文件。还有我希望Calibration库的函数需要通过单元测试，单元测试我打算使用Googletest，一个很出名的单元测试库。

根据上述分析，我需要为Calibration库引入OpenCV、YAML-Cpp、Googletest三个第三方库，以实现我对第三方功能实现的引用。

# 使用第三方库源码作为项目导入

直接将依赖的第三方库以源码形式加入到Calibration的依赖中，可以达到在构建Calibration库时连带构建依赖的第三方库，这样有几个优点：

1. 不需要在运行库的本地环境上安装依赖的静态库`.a`或动态库`.so`文件；
2. 不需要纠结运行环境上已安装的依赖库版本是否和Calibration库兼容；
3. 不需要在构建Calibration库时从网上拉取依赖库源码用于同时构建。

对于Googletest库和yaml-cpp库，我打算将它们的源码加入到Calibration项目中，并且由于它们同样使用CMake构建，因此我可以很方便将它们集成到我的CMake配置文件中，当我构建Calibration库时，它们也同时会被构建。

依赖库的源代码在项目根目录的third-party目录中存放。

同时，我需要使用一个单独的.cmake配置文件，用来描述我要怎样从third-party目录获取第三方库源码，并且将它们作为一个独立的构建目标，然后在我构建Calibration库时将这些第三方库目标同时构建。因此，在cmake目录我编写了一个`third_party_import.cmake`文件，来完成上述操作。同时，在项目根目录的`CMakeLists.txt`中，将这个文件引入，像这样：

```cmake
# CMakeLists.txt

# include 3rd party library
include(${PROJECT_SOURCE_DIR}/cmake/third_party_import.cmake)
```

在third_party_import.cmake文件中，我将以从third-party目录中加入Googletest库为例描述如何通过配置文件导入第三方库的源码并新增构建目标。Googletest库的源代码位于`third-party/googletest-1.8.0`目录中。

我打算使用cmake附带的`FetchContent`模块导入Googletest项目。FetchContent在CMake  v3.11版本被引入，因此为了使用这个功能，`cmake_minimum_required`描述的Cmake版本不能低于3.11。FetchContent不仅可以从远程仓库（例如Git、SVN等）拉取第三方项目源码并加入到现存项目中，同时也可以从本地文件夹获取第三方项目源码。为了在本地获取Googletest项目源码，可以这样写：

```cmake
# cmake/third_party_import.cmake

include(FetchContent)

FetchContent_Declare(
        googletest
        SOURCE_DIR ${PROJECT_SOURCE_DIR}/third-party/googletest-1.8.0
)
FetchContent_GetProperties(googletest)
if (NOT googletest_POPULATED)
    FetchContent_Populate(googletest)
    add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})
endif ()
```

为了使用FetchContent模块，首先要将其include进来。在`FetchContent_Declare`中，我声明了一个名叫googletest的CMake project，`SOURCE_DIR`为该项目源代码所在位置。如果不显式声明`BINARY_DIR`属性来制定项目构建的二进制文件所在位置，那么默认的构建位置位于`PROJECT_BINARY_DIR/_deps/<project_name>-build`目录中。

然后`FetchContent_GetProperties <project_name>`操作读取项目的CMakeList配置文件，如果读取成功会将`<project_name>_POPULATED`字段设为True，那么就可以开始构建了。在构建前还可以用`SET`或`OPTION`操作构建的属性，根据需求设置某个构建选项，比如关闭构建项目的测试文件等。

当执行`FetchContent_Populate <project_name> `时，就开始构建第三方库项目了。此时项目的源代码目录被设置为`<project_name>_SOURCE_DIR`变量，二进制文件位于`<project_name>_BINARY_DIR`变量，就可以使用`add_subdirectory`操作将第三方项目导入到我们的Calibration库项目了。

`add_subdirectory(${<project_name>_SOURCE_DIR} ${<project_name>_BINARY_DIR})`通过显式制定第三方项目的源代码路径和二进制文件目录路径，来引入一个项目。导入成功后会设置第三方项目的头文件目录路径和link target名称，就可以在我的Calibration项目包含头文件和连接目标了。

google-test项目的连接目标是`gtest`和`gtest_main`，如果我要在我自己写的目标链接上google-test，那么我可以这样写：

```cmake
target_link_libraries(<target_name> gtest gtest_main)
```

在CMake版本2.8.11之后，通过上述操作导入的第三方目标会自动将自身的头文件目录路径加入到我们的CMake项目头文件搜索路径，但是在早于2.8.11的CMake版本，需要我们手动将头文件目录加入到搜索路径中：

```cmake
# 以google test为例：
# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()
```

小结：在自己的CMake项目要使用本地源代码方式引入第三方CMake项目，可以使用`FetchContent`引入第三方CMake项目，让其自动生成链接目标，自动将第三方项目的头文件导入到自己的CMake项目中的头文件搜索路径，就可以很方便地在自己的源代码中#include头文件，和构建时使用`target_link_libraries`链接第三方库的目标了。

# 导入本机安装的第三方库

TODO

# 小结

TODO