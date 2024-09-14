# 智能物流搬运小车
本项目是中国大学生工程实践与创新能力大赛中“智能+”赛道的智能物流搬运赛项。  
Collaborators:[CY](https://github.com/CycleYerik)

## Contents
- [CCC_crazy_carrier_car](./CCC_crazy_carrier_car): 项目的主程序代码工程，基于CubeMX生成，使用HAL库函数和Keil5
    - [car_control](./CCC_crazy_carrier_car/car_control): 小车的运动控制、机械臂控制等进行功能实现和封装
    - [screen](./CCC_crazy_carrier_car/screen): 显示屏驱动和显示函数，便于调试
    - [sensor](./CCC_crazy_carrier_car/sensor): 传感器驱动和数据处理
    - [servo](./CCC_crazy_carrier_car/servo): 机械臂舵机相关驱动
    - [motor](./CCC_crazy_carrier_car/motor): 步进电机相关驱动
    - [Core](./CCC_crazy_carrier_car/Core): 主程序代码main.c所在
    - 其余文件夹和文件为CubeMX生成的相关工程文件
- [OpenCV](./OpenCV): 项目的图像处理代码工程，基于树莓派，使用OpenCV




## Usage

### 注意事项  
1. 本项目使用的主控为STM32F407Vet6，使用CubeMX生成工程，使用Keil5进行编译和烧录。自己建立工程进行调试时相对应的时钟配置等可参考本工程
2. 项目将各个模块分开管理，各自调试好驱动，实现功能，给出函数的调用接口或者封装好的功能函数，在主程序中直接调用，这样便于各自模块的调试和管理，最后的main函数里也调用的都是封装好的功能函数，这样代码逻辑也较为清晰。同时建议在各个文件夹中完善README.md文件，给出各个模块的使用方法和接口，说清楚各个函数的作用和参数，而不是就一个.c和.h文件。
3. 各个模块最好能在模块文件中就实现了各种配置和初始化，而不是还要在cubeMX中进行配置。
4. 在car_control文件夹中给出了.c和.h文件的编写模板，可以参考这个模板进行编写，方便后续的调用。尤其注意#ifndef之类的作用。
5. keil编译工程时可能出现中文注释变成乱码的问题，建议自行上网查找解决方法或者干脆用英文注释，但是务必保证用英语能够清晰表达。


## 碎碎念
1. 别问为什么用keil和CubeMX而不用CubeIDE这种明显更现代的平台，问就是祖宗之法不可变。
2. 