/**
 * @file basic_template_and_test.c
 * @author CY
 * @brief  这是.c文件的基本模板，要求尽量在文件的开头添加描述和使用事项，各个函数也加入简要描述,同时函数命名尽量清晰
 * @version 0.1
 * @date 2024-09-14
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#include "basic_template_and_test.h" //引用头文件


// 如下图所示的格式，只要在函数名的上一行输入三个/,就会自动生成函数的注释

/// @brief 测试板载的LED灯，分别为PC5和PB2，每次调用函数都会切换一次LED灯亮灭
/// @param  如果有参数则说明参数的含义
void test_STM32_led(void)
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
}

