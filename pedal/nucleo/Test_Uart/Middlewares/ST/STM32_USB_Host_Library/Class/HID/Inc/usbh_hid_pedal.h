/**
  ******************************************************************************
  * @file    usbh_hid_pedal.h
  * @author  MCD Application Team
  * @brief   This file contains all the prototypes for the usbh_hid_pedal.c
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive  ----------------------------------------------*/
#ifndef __USBH_HID_PEDAL_H
#define __USBH_HID_PEDAL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbh_hid.h"

/** @addtogroup USBH_LIB
  * @{
  */

/** @addtogroup USBH_CLASS
  * @{
  */

/** @addtogroup USBH_HID_CLASS
  * @{
  */

/** @defgroup USBH_HID_PEDAL
  * @brief This file is the Header file for usbh_hid_pedal.c
  * @{
  */


/** @defgroup USBH_HID_PEDAL_Exported_Types
  * @{
  */

typedef struct _HID_PEDAL_Info
{
  uint8_t x;
  uint8_t y;
  uint8_t z;
}
HID_PEDAL_Info_TypeDef;

/**
  * @}
  */

/** @defgroup USBH_HID_PEDAL_Exported_Defines
  * @{
  */
#ifndef USBH_HID_PEDAL_REPORT_SIZE
#define USBH_HID_PEDAL_REPORT_SIZE                       0x3U
#endif /* USBH_HID_PEDAL_REPORT_SIZE */
/**
  * @}
  */

/** @defgroup USBH_HID_PEDAL_Exported_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup USBH_HID_PEDAL_Exported_Variables
  * @{
  */
/**
  * @}
  */

/** @defgroup USBH_HID_PEDAL_Exported_FunctionsPrototype
  * @{
  */
USBH_StatusTypeDef USBH_HID_PedalInit(USBH_HandleTypeDef *phost);
HID_PEDAL_Info_TypeDef *USBH_HID_GetPedalInfo(USBH_HandleTypeDef *phost);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBH_HID_PEDAL_H */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

