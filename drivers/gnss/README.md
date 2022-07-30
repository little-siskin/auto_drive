## Introduction

This is a driver for GNSS devices, such as NovAtel, Applanix, and u-blox. A GNSS device usually
includes a GNSS receiver, an IMU, an interface to a wheel encoder, and a fusion engine that combines
information from those sensors.---这是GNSS设备（如Novatel、Applanix和U-Blox）的驱动程序。GNSS设备通常包括一个GNSS接收器、一个IMU、一个与车轮编码器的接口，以及一个融合引擎，该引擎结合了来自这些传感器的信息。

## Purpose

We aim to the following in this driver.
- Clear C++ code, better code structure, and better performance (than the open-source NovAtel driver).清晰的C++代码，代码结构更好，性能更好（比开源-NoToelDead）。
- Publish sensor-independent protobuf messages such as sensor/gps, sensor/imu, sensor/ins, sensor/wheel.
- Support various sensors: NovAtel, u-blox, STIM300, wheel encoder, etc.
- Log and replay raw data. The log will be used in IE post-processing.
- Support streaming RTK correction to the GPS receiver.---支持对GPS接收机进行流式RTK校正。

## Design

The driver has two nodelets: stream nodelet and parser nodelet. The stream nodelet is in charge of
communication between host PC and the device, as well as grabbing RTK data from a NTRIP caster. The
parser nodelet subscribes the raw data from the stream nodelet, parses the data, and publishes
protobuf messages.---驱动程序有两个节点：流节点和分析器节点。流节点负责主机和设备之间的通信，以及从NTRIP主机获取RTK数据。解析器nodelet从流nodelet订阅原始数据，解析数据并发布protobuf消息。
CNtripcenter软件是基于NTRIP协议的RTCM数据网络分发软件。它可以通过互联网访问和共享全球数据。cntripcenter软件接收各种数据源，直接或间接向客户端发送CORS站数据，完成RTK差分定位，也可以虚拟参数观测数据，实现真正的网络RTK功能。

## Input

- data generated from gnss devices, such as NovAtel, support tcp/usb/udp/ntrip connect method.

## Output

- gnss status
- ins status
- stream status
- imu data
- localization data

## Configuration
We use a protobuf to store all the configuration the driver needs.---我们使用protobuf来存储驱动程序需要的所有配置。
Configure file is stored in path
 `share/gnss_driver/conf/` which is quoted by gnss_driver.launch. File gnss_driver.launch is stored in path
 `share/gnss_driver/launch/`.
When use gnss_driver, the following should be attended.
- Now the location use UTM projection, must check zone id configure in gnss_driver.launch.---现在该位置使用UTM投影，必须在gnss_driver.launch中检查区域ID配置。
- Lever arm distance check.---杠杆臂距离检查。
- Confirm imu install method, this affect vehicle frame and orientation compute.---确认IMU安装方法，这影响到车架和定位计算。
