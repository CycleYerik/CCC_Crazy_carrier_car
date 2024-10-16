# mpu6050六轴陀螺仪

## 概述
完全参照([https://www.iotword.com/27324.html#:~:text=%E5%89%8D%E8%A8%80%20%E5%88%9A%E5%AD%A6%E4%BA%86STM3](https://www.iotword.com/27324.html#:~:text=%E5%89%8D%E8%A8%80%20%E5%88%9A%E5%AD%A6%E4%BA%86STM3))进行的移植，能实现俯仰、横滚、偏航角度的数据读取。**但是温漂太大，基本无法使用，故该驱动基本废弃**

## 使用
- 在cubemx中配置I2C1，引脚为PB8和PB9
- 在mian.c中添加`#include "my_mpu6050.h"`
- 在mian函数中进行初始化，`MPU6050_DMP_init();`,初始化成功则返回0，不成功则可以接入串口调试器查看错误信息
- 调用`MPU6050_DMP_Get_Date(&pitch, &roll, &yaw);`即可获取俯仰、横滚、偏航角度

## 已完成
- 读取原始六轴数据
- 读取俯仰、横滚、偏航角度


## 未完成
- 读取温度