# GSLAM (A General SLAM Framework and BenchMark)

[![Build Status](https://travis-ci.org/zdzhaoyong/GSLAM.svg?branch=master)](https://travis-ci.org/zdzhaoyong/GSLAM)
[![License](https://img.shields.io/badge/license-BSD--2--Clause-blue.svg)](./LICENSE)
[![Version](https://img.shields.io/github/release/zdzhaoyong/GSLAM.svg)](https://github.com/zdzhaoyong/GSLAM/releases)

SLAM技术最近取得了许多成功，吸引了高科技公司的注意。然而，如何统一现有或新兴算法的接口，有效地进行速度、健壮性和可移植性方面的基准测试，仍然是一个难题。本文提出了一种新的SLAM平台GSLAM，它不仅提供了评估功能，而且为研究人员快速开发SLAM系统提供了有用的工具。的核心贡献是为研究和商业应用提供一个通用的、跨平台的、完全开放源码的SLAM接口，旨在在统一的框架中处理与输入数据集、SLAM实现、可视化和应用程序的交互。通过这个平台，用户可以通过插件的形式实现自己的功能，以获得更好的性能，进一步推动应用程序向SLAM的实际应用发展。带有文档wiki的GSLAM源代码已经发布，可以在GitHub上找到。

主要贡献

自20世纪80年代以来，同步定位与映射（SLAM）一直是计算机视觉与机器人领域的研究热点[3,10,14]。SLAM为许多需要实时导航的应用提供了基本功能，如机器人、无人机（UAV）、自动驾驶以及虚拟和增强现实。近年来，SLAM技术发展迅速，提出了多种SLAM系统，包括单目SLAM系统（基于关键点的SLAM系统[12,37,49]、直接的SLAM系统[15,16,53]和半直接的SLAM系统[22,23]）、多传感器SLAM系统（RGBD系统[7,36,68]、立体的SLAM系统[17,23,51]和惯性辅助的SLAM系统[45,56,66]）、基于学习的SLAM系统（监督的SLAM系统[6，55,67]和无监督方法[71,72]。

随着SLAM技术的迅速发展，几乎所有的研究者都把注意力放在了自己SLAM系统的理论和实现上，使得交换思想变得困难，并且不容易将实现移植到其他系统上。妨碍了快速适用于各种工业领域。目前，SLAM系统的实现方式很多，如何有效地进行速度、健壮性和可移植性方面的基准测试仍然是一个问题。最近，Nardi等人[52]和Bodin等人[4]提出了统一的SLAM基准系统，以进行定量、可比和可验证的实验研究，研究不同SLAM系统之间的权衡。通过这些系统，利用数据集和度量评估模块可以方便地进行评估实验。

由于这些系统只提供评估基准，有可能建立一个平台，服务于SLAM算法的整个生命周期，包括开发、评估和应用阶段。基于SLA的深度学习近年来取得了显著的进展，有必要建立一个既支持C++又支持Python的平台，更好地支持基于SLAM系统的几何和深度学习的集成。本文介绍了一种新的SLAM平台，它不仅提供了评估功能，而且为研究人员快速开发自己的SLAM系统提供了有用的工具。通过该平台，用户可以直接使用常用函数或创建自己的函数来实现自己的项目，从而获得更好的性能。希望这个平台能进一步推动SLAM系统的实际应用。总之，这项工作的主要贡献如下：

1） 为研究和商业应用提供了一个通用的、跨平台的、完全开放源码的SLAM平台，这超出了以前的基准。SLAM接口由几个轻量级的、无依赖项的头文件组成，这使得在一个统一的框架中与不同的数据集、SLAM算法和带有插件表单的应用程序进行交互变得容易。此外，JavaScript和Python也被提供给基于web和基于深度学习的SLAM应用程序。

2）在GSLAM平台中，引入了三个优化模块作为工具类，包括估计器、优化器和词汇表。估计器的目的是提供一个封闭形式的求解器的集合，涵盖所有有趣的情况和稳健样本一致性（RANSAC）；优化器的目的是为流行的非线性SLAM问题提供一个单一的接口；词汇表的目的是提供一个有效和便携式的词汇袋实现与多线程和SIMD优化的地方再电离。 3）得益于上述接口，在一个统一的框架中实现并评估了现有数据集、SLAM实现和可视化应用程序的插件，未来新兴基准或应用程序可以更容易地集成。

General SLAM Framework

GSLAM的核心工作是提供一个通用的SLAM接口和框架。为了更好的体验，界面被设计成轻量级的，它由几个头组成，并且仅依赖于C++ 11标准库。基于该接口，支持JavaScript、Python等脚本语言。在这一部分中，介绍了GSLAM框架，并简要介绍了几个基本接口类。 
框架概述

GSLAM的框架如图1所示，一般来说，接口的目的是处理三个部分的交互：


4. SLAM实现实用程序

为了简化SLAM系统的实现，GSLAM提供了一些实用类。本节将简要介绍三个优化模块：估计器、优化器和词汇表。

4.1. 估计器

纯几何计算仍然是一个需要鲁棒精确实时解的基本问题。经典的视觉SLAM算法[22,37,49]和现代的视觉惯性解[45,56,66]都依赖于几何视觉算法进行初始化、重新定位和环路闭合。OpenCV[5]提供了几个几何图形算法和Kneip提出了一个几何视觉工具箱OpenGV[39]，该工具箱仅限于摄像机姿态计算。GSLAM的估计器旨在提供一组用稳健样本一致性（RANSAC）[19]方法覆盖所有有趣案例的闭式解算器。表2列出了估计器支持的算法。根据观察结果，它们被分为三类。利用二维匹配估计外极或单应约束，并从中分解相对位姿。对于单目或多目相机系统，二维-三维对应关系被用来估计中心或非中心绝对位姿，这是著名的PnP问题。还支持三维几何函数，如平面拟合和估计两点云的SIM3变换。大多数算法都是根据线性代数库的特征值来实现的，对于大多数平台来说，特征值只是报头，而且很容易实现。

计算性能评估包括内存使用率、malloc数、CPU使用率和每帧统计使用的时间，如图4所示。结果表明，SVO占用的内存、CPU资源最少，速度最快。所有的成本都保持稳定，因为SVO是一个可视的里程表，并且在实现中只维护一个本地地图。DSO内存块数较少，但占用的内存超过100MB，增长缓慢。DSO的一个问题是当帧数小于500时，处理时间急剧增加，另外，关键帧的处理时间甚至更长。ORBSLAM占用的CPU资源最多，计算时间稳定，但由于bundle调整使用了G2O库，没有采用增量优化方法，内存利用率增长较快，分配和释放了大量内存块。里程表轨迹评估如图5所示。如我们所见，SVO速度更快，但漂移更大，而ORBSLAM在绝对位姿误差（APE）方面达到最佳精度。也提供了相对位姿误差（RPE），但由于图中，补充材料中提供了更多的实验结果。由于集成评估是一个可插入的插件应用程序，因此可以使用更多的评估指标（如pointcloud的精度）重新实现它。

4.2 优化器

非线性优化是最先进的几何SLAM系统的核心部分。由于Hessian矩阵的高维性和稀疏性，采用图结构对SLAM的复杂估计问题进行建模。为了解决一般图优化问题，提出了Ceres[1]、G2O[43]和GTSAM[13]等框架。这些框架被不同的SLAM系统广泛使用。ORB-SLAM[49,51]，SVO[22,23]使用G2O进行束调整和位姿图优化。OKVIS[45]，VINS[56]使用Ceres进行带IMU因子的图优化，并使用滑动窗口控制计算复杂度。Forster等人[21]提出了一种基于SVO的可视化初始方法，并用GTSAM实现了后端。GSLAM的优化器旨在为大多数非线性SLAM问题（如PnP求解器、束调整、位姿图优化）提供一个统一的接口。基于Ceres库实现了一个通用的插件。对于一个特殊的问题，比如包调整，一些更有效的实现，比如PBA[70]和ICE-BA[46]也可以作为插件提供。通过使用优化器实用程序，开发人员可以使用统一的接口访问不同的实现，特别是对于基于深度学习的SLAM系统。


4.3 词汇

位置识别是SLAM重定位和环路检测的重要组成部分。单词包（BoW）方法由于其高效性和性能在SLAM系统中得到了广泛的应用。FabMap[11][30]提出了一种基于位置外观的概率识别方法，RSLAM[47]，LSD-SLAM[16]使用了这种方法。由于使用了诸如SIFT和SURF这样的浮动描述符，DBoW2[25]构建了一个用于训练和检测的词汇树，它同时支持二进制和浮动描述符。Rafael提出了DBoW2的两个改进版本DBoW3和FBoW[48]，简化了界面，加快了训练和加载速度。在ORB-SLAM[49]采用ORB[58]描述符并使用DBoW2进行环路检测[50]、重新定位和快速匹配之后，ORB-SLAM2[51]、VINS Mono[56]和LDSO[26]等多种SLAM系统使用DBoW3进行环路检测。它已经成

为实现SLAM系统位置识别的最流行的工具。受上述工作的启发，GSLAM实现了DBoW3词汇表的header-only实现，具有以下特性：
删除OpenCV依赖性，所有功能都在一个单独的头标题中实现，只依赖于C++ 11。

结合了DBoW2/3和FBoW[48]的优点，这两种方法都非常快速且易于使用。提供了类似于DBoW3的接口，并使用SSE和AVX指令加速二进制和浮点描述符。
我们提高了存储的利用率，加快了加载、保存或训练词汇以及从图像特征到弓向量的转换的速度。

## 1. Introduction

Wiki: https://github.com/zdzhaoyong/GSLAM/wiki
API Documentation: https://zdzhaoyong.github.io/GSLAM/html

### 1.1. What is GSLAM?
GSLAM is aimed to provide a general open-source SLAM framework and benchmark with following features :

-> 1. Share the same API while maintain compatibility with different SLAM systems (such as feature based or direct methods).

-> 2. Support Monocular, Stereo, RGB-D or any custom input types (SAR, IMU, GPS and so on).

-> 3. Provide high efficient implementations of SLAM util classes like SO3, SE3, Camera, IMU, GPS, Bundle and so on.

-> 4. Support other features like coorperation SLAM to build a singular map.

-> 5. Provide benchmark tools for SLAM performance evaluation, make it easy to compare between SLAM systems.

### 1.2. What we can do with GSLAM?
1. *For SLAM developers* : Everyone can develop their own SLAM implementation based on GSLAM and publish it as a plugin with open-source or not. 
2. *For SLAM users* : Applications are able to use different SLAM plugins with the same API without recompilation and implementations are loaded at runtime.

## 2. Compilation and Install

### 2.1. Compile on linux (Tested in Ubuntu 14.04 and 16.04)

#### 2.1.1 Install dependency

**Qt** : REQUIRED, Used by the gslam GUI

```
sudo apt-get install libqt4-dev
```

**OpenCV** : Optional, Used by some dataset plugins and EstimatorOpenCV

```
sudo apt-get install libopencv-dev 
```


#### 2.1.2 Compile and insall GSLAM

```
mkdir build;cd build;
cmake ..;make;sudo make install
```

### 2.2 Compile on windows
Compile with CMake now is supported.

## 3. Start with GSLAM

## 3.1. Basic usages

Test modules with google test:

```
gslam Act=Tests --gtest_filter=*
```

Run a slam system with datasets:

```
gslam Dataset=(dataset file) SLAM=(the slam plugin)
```

## 3.2. Supported Datasets

GSLAM now implemented serveral plugins for public available datasets. It is very easy to play different datasets with parameter "Dataset" setted:

```
# Play kitti with monocular mode
gslam Dataset=<dataset_path>/odomentry/color/00/mono.kitti

# Play kitti with stereo mode
gslam Dataset=<dataset_path>/odomentry/color/00/stereo.kitti

# Play TUM RGBD Dataset (associate.txt file prepared)
gslam Dataset=<dataset_path>/rgbd_dataset_freiburg1_360/.tumrgbd

# Play TUM Monocular (images unziped)
gslam Dataset=<dataset_path>/calib_narrowGamma_scene1/.tummono

# Play EuRoC Dataset with IMU frames
gslam Dataset=<dataset_path>/EuRoC/MH_01_easy/mav0/.euroc

# Play NPU DroneMap Dataset
gslam Dataset=<dataset_path>/DroneMap/phantom3-village/phantom3-village-kfs/.npudronemap
gslam Dataset=<dataset_path>/DroneMap/phantom3-village/phantom3-village-unified/.npudronemap
```

The datasets are default to be played on realtime, and the play speed can be controled with "PlaySpeed":

```
gslam Dataset=<dataset_path>/odomentry/color/00/mono.kitti PlaySpeed=2.
```

The following dataset plugins are now implemented:

| Name    |    Channels        |   Description    |
| ------- |:------------------:|:-------------:|
| [KITTI](http://www.cvlibs.net/datasets/kitti/)   | Stereo,Pose        |               |
| [TUMMono](https://vision.in.tum.de/data/datasets/mono-dataset) | Monocular          | |
| [TUMRGBD](https://vision.in.tum.de/data/datasets/rgbd-dataset) | RGBD,Pose          ||
| [EuRoc](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)   | IMU,Stereo         ||
| [NPUDroneMap](http://zhaoyong.adv-ci.com/downloads/npu-dronemap-dataset/)| GPS,Monocular   ||
| CVMono | Monocular           | Online camera or video dataset using opencv.|

Users can also [implement dataset plugins by own](./doc/md/dataset.md).

### 3.3. Implemented SLAM plugins
| Name        |  ScreenShot  | Description  |
| ------- |:------:|:-------------:|
| [DSO](https://github.com/JakobEngel/dso)     |  <img src="./doc/images/gslam_dso.gif" width = "50%" /> | [code](https://github.com/pi-gslam/GSLAM-DSO) |
| [ORBSLAM](https://github.com/raulmur/ORB_SLAM) |  <img src="./doc/images/gslam_orbslam.gif" width = "50%" />| [code](https://github.com/pi-gslam/GSLAM-ORBSLAM) |
| [SVO](https://github.com/uzh-rpg/rpg_svo) |  <img src="./doc/images/gslam_svo.gif" width = "50%" />| [code](https://github.com/pi-gslam/GSLAM-SVO) |
| [TheiaSfM](http://www.theia-sfm.org/) |  <img src="./doc/images/gslam_theiaSfM.png" width = "50%" />| [code](https://github.com/zdzhaoyong/GSLAM-TheiaSfM) |

### 3.4. Configuration with Svar
More parameters can be setted with Svar at file *.cfg.
See more details of Svar at [PILBASE](https://github.com/zdzhaoyong/PIL2/blob/master/apps/SvarTest/README.md).

## 4. Contacts

YongZhao: zd5945@126.com

ShuhuiBu: bushuhui@nwpu.edu.cn

## 5. License

The GSLAM library is licensed under the BSD license. Note that this text refers only to the license for GSLAM itself, independent of its optional dependencies, which are separately licensed. Building GSLAM with these optional dependencies may affect the resulting GSLAM license.

```
Copyright (c) 2018 Northwestern Polytechnical University, Yong Zhao. All rights reserved.

This software was developed by the Yong Zhao at Northwestern Polytechnical University.

All advertising materials mentioning features or use of this software must display
the following acknowledgement: This product includes software developed by Northwestern Polytechnical University, PILAB.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must
display the following acknowledgement: This product includes software developed
by Northwestern Polytechnical University and its contributors.
4. Neither the name of the University nor the names of its contributors may be
used to endorse or promote products derived from this software without specific
prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

