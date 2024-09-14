#ifndef BASIC_TEMPLATE_AND_TEST_H
#define BASIC_TEMPLATE_AND_TEST_H

/**
 * @brief 所有的.h文件都以这个为模板，尽量把各种要调整的参数用#define，方便后续修改。所需要的include也在这里添加。尤其注意.h文件#ifndef等的使用，可以自己去查一下，务必了解其作用
 * 
 * 
 */




// 各种要调整的参数（比如转速、距离等）都用#define定义，方便后续修改
#define test_GPIO GPIOC


void test_STM32_led(void);

#endif