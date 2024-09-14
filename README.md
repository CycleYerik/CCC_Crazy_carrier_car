# 智能物流搬运小车
本项目是中国大学生工程实践与创新能力大赛中“智能+”赛道的智能物流搬运赛项。  
Collaborators:[CY].(https://github.com/CycleYerik)

## Contents
- [CCC_crazy_carrier_car].(./CCC_crazy_carrier_car): 项目的主程序代码工程，基于CubeMX生成，使用HAL库函数和Keil5
    - .[car_control].(./CCC_crazy_carrier_car/car_control): 小车的运动控制、机械臂控制等进行功能实现和封装
    - .[screen].(./CCC_crazy_carrier_car/screen): 显示屏驱动和显示函数，便于调试
    - .[sensor].(./CCC_crazy_carrier_car/sensor): 传感器驱动和数据处理
    - .[servo].(./CCC_crazy_carrier_car/servo): 机械臂舵机相关驱动
    - .[motor].(./CCC_crazy_carrier_car/motor): 步进电机相关驱动
    - .[Core].(./CCC_crazy_carrier_car/Core): 主程序代码main.c所在
    - 其余文件夹和文件为CubeMX生成的相关工程文件
- [OpenCV].(./OpenCV): 项目的图像处理代码工程，基于树莓派，使用OpenCV


## Usage

### 注意事项  
1. 本项目使用的主控为STM32F407Vet6，使用CubeMX生成工程，使用Keil5进行编译和烧录