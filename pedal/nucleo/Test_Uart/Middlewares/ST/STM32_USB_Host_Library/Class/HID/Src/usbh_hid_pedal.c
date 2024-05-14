/**
  ******************************************************************************
  * @file    usbh_hid_pedal.c
  * @author  MCD Application Team
  * @brief   This file is the application layer for USB Host HID Pedal Handling.
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

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
- "stm32xxxxx_{eval}{discovery}{adafruit}_lcd.c"
- "stm32xxxxx_{eval}{discovery}_sdram.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbh_hid_pedal.h"
#include "usbh_hid_parser.h"


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
  * @brief    This file includes HID Layer Handlers for USB Host HID class.
  * @{
  */

/** @defgroup USBH_HID_PEDAL_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBH_HID_PEDAL_Private_Defines
  * @{
  */
/**
  * @}
  */


/** @defgroup USBH_HID_PEDAL_Private_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup USBH_HID_PEDAL_Private_FunctionPrototypes
  * @{
  */
static USBH_StatusTypeDef USBH_HID_PedalDecode(USBH_HandleTypeDef *phost);

/**
  * @}
  */


/** @defgroup USBH_HID_PEDAL_Private_Variables
  * @{
  */
HID_PEDAL_Info_TypeDef   pedal_info;
uint8_t                  pedal_report_data[USBH_HID_PEDAL_REPORT_SIZE];
uint8_t                  pedal_rx_report_buf[USBH_HID_PEDAL_REPORT_SIZE];

/* Structures defining how to access items in a HID pedal report */

/* Access x coordinate change. */
static const HID_Report_ItemTypedef prop_x =
{
  pedal_report_data + 0U, /*data*/
  8,     /*size*/
  0,     /*shift*/
  0,     /*count (only for array items)*/
  0,     /*signed?*/
  0,     /*min value read can return*/
  0xFFFF,/*max value read can return*/
  0,     /*min vale device can report*/
  0xFFFF,/*max value device can report*/
  1      /*resolution*/
};

/* Access y coordinate change. */
static const HID_Report_ItemTypedef prop_y =
{
  pedal_report_data + 1U, /*data*/
  8,     /*size*/
  0,     /*shift*/
  0,     /*count (only for array items)*/
  0,     /*signed?*/
  0,     /*min value read can return*/
  0xFFFF,/*max value read can return*/
  0,     /*min vale device can report*/
  0xFFFF,/*max value device can report*/
  1      /*resolution*/
};

/* Access y coordinate change. */
static const HID_Report_ItemTypedef prop_z =
{
  pedal_report_data + 2U, /*data*/
  8,     /*size*/
  0,     /*shift*/
  0,     /*count (only for array items)*/
  1,     /*signed?*/
  0,     /*min value read can return*/
  0xFFFF,/*max value read can return*/
  0,     /*min vale device can report*/
  0xFFFF,/*max value device can report*/
  1      /*resolution*/
};


/**
  * @}
  */


/** @defgroup USBH_HID_PEDAL_Private_Functions
  * @{
  */

/**
  * @brief  USBH_HID_PedalInit
  *         The function init the HID pedal.
  * @param  phost: Host handle
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_HID_PedalInit(USBH_HandleTypeDef *phost)
{
  uint32_t i;
  HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

  pedal_info.x = 0U;
  pedal_info.y = 0U;
  pedal_info.z = 0U;

  for (i = 0U; i < sizeof(pedal_report_data); i++)
  {
    pedal_report_data[i] = 0U;
    pedal_rx_report_buf[i] = 0U;
  }

  if (HID_Handle->length > sizeof(pedal_report_data))
  {
    HID_Handle->length = (uint16_t)sizeof(pedal_report_data);
  }
  HID_Handle->pData = pedal_rx_report_buf;

  if ((HID_QUEUE_SIZE * sizeof(pedal_report_data)) > sizeof(phost->device.Data))
  {
    return USBH_FAIL;
  }
  else
  {
    USBH_HID_FifoInit(&HID_Handle->fifo, phost->device.Data, (uint16_t)(HID_QUEUE_SIZE * sizeof(pedal_report_data)));
  }

  return USBH_OK;
}

/**
  * @brief  USBH_HID_GetPedalInfo
  *         The function return pedal information.
  * @param  phost: Host handle
  * @retval pedal information
  */
HID_PEDAL_Info_TypeDef *USBH_HID_GetPedalInfo(USBH_HandleTypeDef *phost)
{
  if (USBH_HID_PedalDecode(phost) == USBH_OK)
  {
    return &pedal_info;
  }
  else
  {
    return NULL;
  }
}

/**
  * @brief  USBH_HID_PedalDecode
  *         The function decode pedal data.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_PedalDecode(USBH_HandleTypeDef *phost)
{
  HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

  if ((HID_Handle->length == 0U) || (HID_Handle->fifo.buf == NULL))
  {
    return USBH_FAIL;
  }
  /*Fill report */
  if (USBH_HID_FifoRead(&HID_Handle->fifo, &pedal_report_data, HID_Handle->length) == HID_Handle->length)
  {
    /*Decode report */
    pedal_info.x = (uint8_t)HID_ReadItem((HID_Report_ItemTypedef *) &prop_x, 0U);
    pedal_info.y = (uint8_t)HID_ReadItem((HID_Report_ItemTypedef *) &prop_y, 0U);
    pedal_info.z = (uint8_t)HID_ReadItem((HID_Report_ItemTypedef *) &prop_z, 0U);

    return USBH_OK;
  }
  return   USBH_FAIL;
}

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


/**
  * @}
  */
