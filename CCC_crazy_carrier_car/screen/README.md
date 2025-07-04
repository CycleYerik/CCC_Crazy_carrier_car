# 串口屏

## 概述
采用淘晶池的串口屏。借助stm32的串口实现通信，通过电脑上位机进行显示界面的设计，借助usb转ttl模块进行固件的烧录。支持触屏功能，多种显示控件，支持自定义控件。

## 使用
重定义了printf函数，可以直接用printf进行发送数据。
- 波特率为9600
- 发送数据需按照一定的格式，具体参考串口HMI指令集
- ```c
  printf("page 0\xff\xff\xff"); //切换到0号页面
  printf("t0.txt=\"%.2f\"\xff\xff\xff", yaw); //在t0文本框中显示yaw的值,\xff\xff\xff为结束符
  ```
- 在上位机中设计好界面，比如某个按钮按下后发送某个数据，然后在stm32中接收到数据后进行相应的操作

## 已完成
- 基本的显示功能（显示数据、字符串等在串口屏上）
- 按钮按下后发送数据实现特定功能

## 未完成
- 界面切换以实现不同的功能
- 加入更多的控件及功能，比如一键启动、测试转速、显示转速和电压等
