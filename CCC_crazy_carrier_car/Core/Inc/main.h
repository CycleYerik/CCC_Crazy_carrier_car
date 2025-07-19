/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#include <stdint.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef enum {FALSE = 0,TRUE = 1} bool;

#define max_data_length 100

/// @brief 物料位置和颜色的关系
typedef struct {
    int left;
    int middle;
    int right;
} material_order;


// 定义一个物料抓放的结构体
typedef struct
{
    int times; // 运行次数
    int run_round_number; //运行3轮还是1轮（测试用）
    int is_avoid_collide; // 是否从侧面过
    int is_load; //是否放完后抓起来 
    int is_pile_up; // 是否码垛
    int is_adjust_without_material;
    int is_none_pile_up_put_adjust; //不进行码垛时放置是否调整
    int is_pile_up_adjust; //码垛时放置是否调整
    int is_get_from_car_plate_update; //从车身载物盘抓取物料时是否进行中心位置更新（用于物料中心与夹爪中心不重合的情况）
    int is_get_from_ground_check; //暂时未启用从地面抓取物料时是否进行空抓判断
} material_get_and_put_struct;

// 定义一个在转盘抓放的结构体
typedef struct
{
    int is_avoid;
    int empty_check;
    int back_check;
} plate_get_struct;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
