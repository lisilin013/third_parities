# third_parties

## 1 Introduction

为了便于个人软件开发，构建了third_parties repo，这个repo里面的packages基本来自于ETH-asl实验室github网站：https://github.com/ethz-asl。

ETH-asl实验室封装的这些第三方库，或者他们自己开发的第三方库，有以下几个优点：

- 封装了`catkin_simple`库，简化了使用cmake开发cpp ros软件包的命令
- 使用`catkin build`编译project
- 将常用的第三方软件包使用cmake  `ExternalProject_Add` 的方式加入进来，固定版本，不依赖开发者系统中的软件版本

此外，为了加快下载，我将`ExternalProject_Add`中的URL网络链接替换成了本地文件地址。

## 2 Build
- 使用third_parties repo之前，先要安装catkin_tools工具，[link](https://catkin-tools.readthedocs.io/en/latest/installing.html)。

- download and build
```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/lisilin013/third_parities.git
cd ..
catkin build
```

