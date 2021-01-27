/**
 ******************************************************************************
 * @file    LSM6DSRSensor.cpp
 * @author  SRA
 * @version V1.0.0
 * @date    March 2020
 * @brief   Implementation of an LSM6DSR Inertial Measurement Unit (IMU) 6 axes
 *          sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/

#include "LSM6DSRSensor.h"


/* Class Implementation ------------------------------------------------------*/

/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 */
LSM6DSRSensor::LSM6DSRSensor(TwoWire *i2c, uint8_t address) : dev_i2c(i2c), address(address)
{
  dev_spi = NULL;
  reg_ctx.write_reg = LSM6DSR_io_write;
  reg_ctx.read_reg = LSM6DSR_io_read;
  reg_ctx.handle = (void *)this;
  acc_is_enabled = 0U;
  gyro_is_enabled = 0U;
}

/** Constructor
 * @param spi object of an helper class which handles the SPI peripheral
 * @param cs_pin the chip select pin
 * @param spi_speed the SPI speed
 */
LSM6DSRSensor::LSM6DSRSensor(SPIClass *spi, int cs_pin, uint32_t spi_speed) : dev_spi(spi), cs_pin(cs_pin), spi_speed(spi_speed)
{
  reg_ctx.write_reg = LSM6DSR_io_write;
  reg_ctx.read_reg = LSM6DSR_io_read;
  reg_ctx.handle = (void *)this;
  dev_i2c = NULL;
  address = 0U;
  acc_is_enabled = 0U;
  gyro_is_enabled = 0U;
}

/**
 * @brief  Configure the sensor in order to be used
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::begin()
{
  if(dev_spi)
  {
    // Configure CS pin
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH); 
  }

  /* Disable I3C */
  if (lsm6dsr_i3c_disable_set(&reg_ctx, LSM6DSR_I3C_DISABLE) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  /* Enable register address automatically incremented during a multiple byte
  access with a serial interface. */
  if (lsm6dsr_auto_increment_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  /* Enable BDU */
  if (lsm6dsr_block_data_update_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  /* FIFO mode selection */
  if (lsm6dsr_fifo_mode_set(&reg_ctx, LSM6DSR_BYPASS_MODE) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  /* Select default output data rate. */
  acc_odr = LSM6DSR_XL_ODR_104Hz;

  /* Output data rate selection - power down. */
  if (lsm6dsr_xl_data_rate_set(&reg_ctx, LSM6DSR_XL_ODR_OFF) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  /* Full scale selection. */
  if (lsm6dsr_xl_full_scale_set(&reg_ctx, LSM6DSR_2g) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  /* Select default output data rate. */
  gyro_odr = LSM6DSR_GY_ODR_104Hz;

  /* Output data rate selection - power down. */
  if (lsm6dsr_gy_data_rate_set(&reg_ctx, LSM6DSR_GY_ODR_OFF) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  /* Full scale selection. */
  if (lsm6dsr_gy_full_scale_set(&reg_ctx, LSM6DSR_2000dps) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }
  
  acc_is_enabled = 0;
  gyro_is_enabled = 0;

  return LSM6DSR_OK;
}

/**
 * @brief  Disable the sensor and relative resources
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::end()
{
  /* Disable both acc and gyro */
  if (Disable_X() != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  if (Disable_G() != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  /* Reset CS configuration */
  if(dev_spi)
  {
    // Configure CS pin
    pinMode(cs_pin, INPUT); 
  }

  return LSM6DSR_OK;
}

/**
 * @brief  Read component ID
 * @param  Id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::ReadID(uint8_t *Id)
{
  if (lsm6dsr_device_id_get(&reg_ctx, Id) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  return LSM6DSR_OK;
}

/**
 * @brief  Enable the LSM6DSR accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Enable_X()
{
  /* Check if the component is already enabled */
  if (acc_is_enabled == 1U)
  {
    return LSM6DSR_OK;
  }

  /* Output data rate selection. */
  if (lsm6dsr_xl_data_rate_set(&reg_ctx, acc_odr) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  acc_is_enabled = 1;

  return LSM6DSR_OK;
}

/**
 * @brief  Disable the LSM6DSR accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Disable_X()
{
  /* Check if the component is already disabled */
  if (acc_is_enabled == 0U)
  {
    return LSM6DSR_OK;
  }

  /* Get current output data rate. */
  if (lsm6dsr_xl_data_rate_get(&reg_ctx, &acc_odr) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  /* Output data rate selection - power down. */
  if (lsm6dsr_xl_data_rate_set(&reg_ctx, LSM6DSR_XL_ODR_OFF) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  acc_is_enabled = 0;

  return LSM6DSR_OK;
}

/**
 * @brief  Get the LSM6DSR accelerometer sensor sensitivity
 * @param  Sensitivity pointer
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Get_X_Sensitivity(float *Sensitivity)
{
  LSM6DSRStatusTypeDef ret = LSM6DSR_OK;
  lsm6dsr_fs_xl_t full_scale;

  /* Read actual full scale selection from sensor. */
  if (lsm6dsr_xl_full_scale_get(&reg_ctx, &full_scale) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  /* Store the Sensitivity based on actual full scale. */
  switch (full_scale)
  {
    case LSM6DSR_2g:
      *Sensitivity = LSM6DSR_ACC_SENSITIVITY_FS_2G;
      break;

    case LSM6DSR_4g:
      *Sensitivity = LSM6DSR_ACC_SENSITIVITY_FS_4G;
      break;

    case LSM6DSR_8g:
      *Sensitivity = LSM6DSR_ACC_SENSITIVITY_FS_8G;
      break;

    case LSM6DSR_16g:
      *Sensitivity = LSM6DSR_ACC_SENSITIVITY_FS_16G;
      break;

    default:
      ret = LSM6DSR_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Get the LSM6DSR accelerometer sensor output data rate
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Get_X_ODR(float *Odr)
{
  LSM6DSRStatusTypeDef ret = LSM6DSR_OK;
  lsm6dsr_odr_xl_t odr_low_level;

  /* Get current output data rate. */
  if (lsm6dsr_xl_data_rate_get(&reg_ctx, &odr_low_level) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  switch (odr_low_level)
  {
    case LSM6DSR_XL_ODR_OFF:
      *Odr = 0.0f;
      break;

    case LSM6DSR_XL_ODR_1Hz6:
      *Odr = 1.6f;
      break;

    case LSM6DSR_XL_ODR_12Hz5:
      *Odr = 12.5f;
      break;

    case LSM6DSR_XL_ODR_26Hz:
      *Odr = 26.0f;
      break;

    case LSM6DSR_XL_ODR_52Hz:
      *Odr = 52.0f;
      break;

    case LSM6DSR_XL_ODR_104Hz:
      *Odr = 104.0f;
      break;

    case LSM6DSR_XL_ODR_208Hz:
      *Odr = 208.0f;
      break;

    case LSM6DSR_XL_ODR_417Hz:
      *Odr = 417.0f;
      break;

    case LSM6DSR_XL_ODR_833Hz:
      *Odr = 833.0f;
      break;

    case LSM6DSR_XL_ODR_1667Hz:
      *Odr = 1667.0f;
      break;

    case LSM6DSR_XL_ODR_3333Hz:
      *Odr = 3333.0f;
      break;

    case LSM6DSR_XL_ODR_6667Hz:
      *Odr = 6667.0f;
      break;

    default:
      ret = LSM6DSR_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSR accelerometer sensor output data rate
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Set_X_ODR(float Odr)
{
  return Set_X_ODR_With_Mode(Odr, LSM6DSR_ACC_HIGH_PERFORMANCE_MODE);
}

/**
 * @brief  Set the LSM6DSR accelerometer sensor output data rate with operating mode
 * @param  Odr the output data rate value to be set
 * @param  Mode the accelerometer operating mode
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Set_X_ODR_With_Mode(float Odr, LSM6DSR_ACC_Operating_Mode_t Mode)
{
  LSM6DSRStatusTypeDef ret = LSM6DSR_OK;
  float newOdr = Odr;
  
  switch (Mode)
  {
    case LSM6DSR_ACC_HIGH_PERFORMANCE_MODE:
    {
      /* We must uncheck Low Power bit if it is enabled */
      lsm6dsr_ctrl6_c_t val;

      if (lsm6dsr_read_reg(&reg_ctx, LSM6DSR_CTRL6_C, (uint8_t *)&val, 1) != LSM6DSR_OK)
      {
        return LSM6DSR_ERROR;
      }

      if (val.xl_hm_mode != 0U)
      {
        val.xl_hm_mode = 0U;
        if (lsm6dsr_write_reg(&reg_ctx, LSM6DSR_CTRL6_C, (uint8_t *)&val, 1) != LSM6DSR_OK)
        {
          return LSM6DSR_ERROR;
        }
      }

      /* ODR should be at least 12.5Hz */
      if (newOdr < 12.5f)
      {
        newOdr = 12.5f;
      }
      break;
    }
    case LSM6DSR_ACC_LOW_POWER_NORMAL_MODE:
    {
      /* We must check the Low Power bit if it is unchecked */
      lsm6dsr_ctrl6_c_t val;

      if (lsm6dsr_read_reg(&reg_ctx, LSM6DSR_CTRL6_C, (uint8_t *)&val, 1) != LSM6DSR_OK)
      {
        return LSM6DSR_ERROR;
      }

      if (val.xl_hm_mode == 0U)
      {
        val.xl_hm_mode = 1U;
        if (lsm6dsr_write_reg(&reg_ctx, LSM6DSR_CTRL6_C, (uint8_t *)&val, 1) != LSM6DSR_OK)
        {
          return LSM6DSR_ERROR;
        }
      }

      /* Now we need to limit the ODR to 208 Hz if it is higher */
      if (newOdr > 208.0f)
      {
        newOdr = 208.0f;
      }
      break;
    }
    default:
      ret = LSM6DSR_ERROR;
      break;
  }

  if(ret == LSM6DSR_ERROR)
  {
    return LSM6DSR_ERROR;
  }

  if (acc_is_enabled == 1U)
  {
    ret = Set_X_ODR_When_Enabled(newOdr);
  }
  else
  {
    ret = Set_X_ODR_When_Disabled(newOdr);
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSR accelerometer sensor output data rate when enabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Set_X_ODR_When_Enabled(float Odr)
{
  lsm6dsr_odr_xl_t new_odr;

  new_odr = (Odr <=    1.6f) ? LSM6DSR_XL_ODR_1Hz6
          : (Odr <=   12.5f) ? LSM6DSR_XL_ODR_12Hz5
          : (Odr <=   26.0f) ? LSM6DSR_XL_ODR_26Hz
          : (Odr <=   52.0f) ? LSM6DSR_XL_ODR_52Hz
          : (Odr <=  104.0f) ? LSM6DSR_XL_ODR_104Hz
          : (Odr <=  208.0f) ? LSM6DSR_XL_ODR_208Hz
          : (Odr <=  417.0f) ? LSM6DSR_XL_ODR_417Hz
          : (Odr <=  833.0f) ? LSM6DSR_XL_ODR_833Hz
          : (Odr <= 1667.0f) ? LSM6DSR_XL_ODR_1667Hz
          : (Odr <= 3333.0f) ? LSM6DSR_XL_ODR_3333Hz
          :                    LSM6DSR_XL_ODR_6667Hz;

  /* Output data rate selection. */
  if (lsm6dsr_xl_data_rate_set(&reg_ctx, new_odr) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  return LSM6DSR_OK;
}

/**
 * @brief  Set the LSM6DSR accelerometer sensor output data rate when disabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Set_X_ODR_When_Disabled(float Odr)
{
  acc_odr = (Odr <=    1.6f) ? LSM6DSR_XL_ODR_1Hz6
          : (Odr <=   12.5f) ? LSM6DSR_XL_ODR_12Hz5
          : (Odr <=   26.0f) ? LSM6DSR_XL_ODR_26Hz
          : (Odr <=   52.0f) ? LSM6DSR_XL_ODR_52Hz
          : (Odr <=  104.0f) ? LSM6DSR_XL_ODR_104Hz
          : (Odr <=  208.0f) ? LSM6DSR_XL_ODR_208Hz
          : (Odr <=  417.0f) ? LSM6DSR_XL_ODR_417Hz
          : (Odr <=  833.0f) ? LSM6DSR_XL_ODR_833Hz
          : (Odr <= 1667.0f) ? LSM6DSR_XL_ODR_1667Hz
          : (Odr <= 3333.0f) ? LSM6DSR_XL_ODR_3333Hz
          :                    LSM6DSR_XL_ODR_6667Hz;

  return LSM6DSR_OK;
}


/**
 * @brief  Get the LSM6DSR accelerometer sensor full scale
 * @param  FullScale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Get_X_FS(int32_t *FullScale)
{
  LSM6DSRStatusTypeDef ret = LSM6DSR_OK;
  lsm6dsr_fs_xl_t fs_low_level;

  /* Read actual full scale selection from sensor. */
  if (lsm6dsr_xl_full_scale_get(&reg_ctx, &fs_low_level) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  switch (fs_low_level)
  {
    case LSM6DSR_2g:
      *FullScale =  2;
      break;

    case LSM6DSR_4g:
      *FullScale =  4;
      break;

    case LSM6DSR_8g:
      *FullScale =  8;
      break;

    case LSM6DSR_16g:
      *FullScale = 16;
      break;

    default:
      ret = LSM6DSR_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSR accelerometer sensor full scale
 * @param  FullScale the functional full scale to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Set_X_FS(int32_t FullScale)
{
  lsm6dsr_fs_xl_t new_fs;

  /* Seems like MISRA C-2012 rule 14.3a violation but only from single file statical analysis point of view because
     the parameter passed to the function is not known at the moment of analysis */
  new_fs = (FullScale <= 2) ? LSM6DSR_2g
         : (FullScale <= 4) ? LSM6DSR_4g
         : (FullScale <= 8) ? LSM6DSR_8g
         :                    LSM6DSR_16g;

  if (lsm6dsr_xl_full_scale_set(&reg_ctx, new_fs) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  return LSM6DSR_OK;
}

/**
 * @brief  Get the LSM6DSR accelerometer sensor raw axes
 * @param  Value pointer where the raw values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Get_X_AxesRaw(int16_t *Value)
{
  axis3bit16_t data_raw;

  /* Read raw data values. */
  if (lsm6dsr_acceleration_raw_get(&reg_ctx, data_raw.u8bit) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  /* Format the data. */
  Value[0] = data_raw.i16bit[0];
  Value[1] = data_raw.i16bit[1];
  Value[2] = data_raw.i16bit[2];

  return LSM6DSR_OK;
}


/**
 * @brief  Get the LSM6DSR accelerometer sensor axes
 * @param  Acceleration pointer where the values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Get_X_Axes(int32_t *Acceleration)
{
  axis3bit16_t data_raw;
  float sensitivity = 0.0f;

  /* Read raw data values. */
  if (lsm6dsr_acceleration_raw_get(&reg_ctx, data_raw.u8bit) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  /* Get LSM6DSR actual sensitivity. */
  if (Get_X_Sensitivity(&sensitivity) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  /* Calculate the data. */
  Acceleration[0] = (int32_t)((float)((float)data_raw.i16bit[0] * sensitivity));
  Acceleration[1] = (int32_t)((float)((float)data_raw.i16bit[1] * sensitivity));
  Acceleration[2] = (int32_t)((float)((float)data_raw.i16bit[2] * sensitivity));

  return LSM6DSR_OK;
}


/**
 * @brief  Get the LSM6DSR ACC data ready bit value
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Get_X_DRDY_Status(uint8_t *Status)
{
  if (lsm6dsr_xl_flag_data_ready_get(&reg_ctx, Status) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  return LSM6DSR_OK;
}


/**
 * @brief  Enable the LSM6DSR gyroscope sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Enable_G()
{
  /* Check if the component is already enabled */
  if (gyro_is_enabled == 1U)
  {
    return LSM6DSR_OK;
  }

  /* Output data rate selection. */
  if (lsm6dsr_gy_data_rate_set(&reg_ctx, gyro_odr) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  gyro_is_enabled = 1;

  return LSM6DSR_OK;
}


/**
 * @brief  Disable the LSM6DSR gyroscope sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Disable_G()
{
  /* Check if the component is already disabled */
  if (gyro_is_enabled == 0U)
  {
    return LSM6DSR_OK;
  }

  /* Get current output data rate. */
  if (lsm6dsr_gy_data_rate_get(&reg_ctx, &gyro_odr) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  /* Output data rate selection - power down. */
  if (lsm6dsr_gy_data_rate_set(&reg_ctx, LSM6DSR_GY_ODR_OFF) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  gyro_is_enabled = 0;

  return LSM6DSR_OK;
}

/**
 * @brief  Get the LSM6DSR gyroscope sensor sensitivity
 * @param  Sensitivity pointer
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Get_G_Sensitivity(float *Sensitivity)
{
  LSM6DSRStatusTypeDef ret = LSM6DSR_OK;
  lsm6dsr_fs_g_t full_scale;

  /* Read actual full scale selection from sensor. */
  if (lsm6dsr_gy_full_scale_get(&reg_ctx, &full_scale) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  /* Store the sensitivity based on actual full scale. */
  switch (full_scale)
  {
    case LSM6DSR_125dps:
      *Sensitivity = LSM6DSR_GYRO_SENSITIVITY_FS_125DPS;
      break;

    case LSM6DSR_250dps:
      *Sensitivity = LSM6DSR_GYRO_SENSITIVITY_FS_250DPS;
      break;

    case LSM6DSR_500dps:
      *Sensitivity = LSM6DSR_GYRO_SENSITIVITY_FS_500DPS;
      break;

    case LSM6DSR_1000dps:
      *Sensitivity = LSM6DSR_GYRO_SENSITIVITY_FS_1000DPS;
      break;

    case LSM6DSR_2000dps:
      *Sensitivity = LSM6DSR_GYRO_SENSITIVITY_FS_2000DPS;
      break;

    case LSM6DSR_4000dps:
      *Sensitivity = LSM6DSR_GYRO_SENSITIVITY_FS_4000DPS;
      break;

    default:
      ret = LSM6DSR_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Get the LSM6DSR gyroscope sensor output data rate
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Get_G_ODR(float *Odr)
{
  LSM6DSRStatusTypeDef ret = LSM6DSR_OK;
  lsm6dsr_odr_g_t odr_low_level;

  /* Get current output data rate. */
  if (lsm6dsr_gy_data_rate_get(&reg_ctx, &odr_low_level) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  switch (odr_low_level)
  {
    case LSM6DSR_GY_ODR_OFF:
      *Odr = 0.0f;
      break;

    case LSM6DSR_GY_ODR_12Hz5:
      *Odr = 12.5f;
      break;

    case LSM6DSR_GY_ODR_26Hz:
      *Odr = 26.0f;
      break;

    case LSM6DSR_GY_ODR_52Hz:
      *Odr = 52.0f;
      break;

    case LSM6DSR_GY_ODR_104Hz:
      *Odr = 104.0f;
      break;

    case LSM6DSR_GY_ODR_208Hz:
      *Odr = 208.0f;
      break;

    case LSM6DSR_GY_ODR_417Hz:
      *Odr = 417.0f;
      break;

    case LSM6DSR_GY_ODR_833Hz:
      *Odr = 833.0f;
      break;

    case LSM6DSR_GY_ODR_1667Hz:
      *Odr =  1667.0f;
      break;

    case LSM6DSR_GY_ODR_3333Hz:
      *Odr =  3333.0f;
      break;

    case LSM6DSR_GY_ODR_6667Hz:
      *Odr =  6667.0f;
      break;

    default:
      ret = LSM6DSR_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSR gyroscope sensor output data rate
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Set_G_ODR(float Odr)
{
  return Set_G_ODR_With_Mode(Odr, LSM6DSR_GYRO_HIGH_PERFORMANCE_MODE);
}

/**
 * @brief  Set the LSM6DSR gyroscope sensor output data rate with operating mode
 * @param  Odr the output data rate value to be set
 * @param  Mode the gyroscope operating mode
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Set_G_ODR_With_Mode(float Odr, LSM6DSR_GYRO_Operating_Mode_t Mode)
{
  LSM6DSRStatusTypeDef ret = LSM6DSR_OK;
  float newOdr = Odr;

  switch (Mode)
  {
    case LSM6DSR_GYRO_HIGH_PERFORMANCE_MODE:
    {
      /* We must uncheck Low Power bit if it is enabled */
      lsm6dsr_ctrl7_g_t val;

      if (lsm6dsr_read_reg(&reg_ctx, LSM6DSR_CTRL7_G, (uint8_t *)&val, 1) != LSM6DSR_OK)
      {
        return LSM6DSR_ERROR;
      }

      if (val.g_hm_mode != 0U)
      {
        val.g_hm_mode = 0U;
        if (lsm6dsr_write_reg(&reg_ctx, LSM6DSR_CTRL7_G, (uint8_t *)&val, 1) != LSM6DSR_OK)
        {
          return LSM6DSR_ERROR;
        }
      }
      break;
    }
    case LSM6DSR_GYRO_LOW_POWER_NORMAL_MODE:
    {
      /* We must check the Low Power bit if it is unchecked */
      lsm6dsr_ctrl7_g_t val;

      if (lsm6dsr_read_reg(&reg_ctx, LSM6DSR_CTRL7_G, (uint8_t *)&val, 1) != LSM6DSR_OK)
      {
        return LSM6DSR_ERROR;
      }

      if (val.g_hm_mode == 0U)
      {
        val.g_hm_mode = 1U;
        if (lsm6dsr_write_reg(&reg_ctx, LSM6DSR_CTRL7_G, (uint8_t *)&val, 1) != LSM6DSR_OK)
        {
          return LSM6DSR_ERROR;
        }
      }

      /* Now we need to limit the ODR to 208 Hz if it is higher */
      if (newOdr > 208.0f)
      {
        newOdr = 208.0f;
      }
      break;
    }
    default:
      ret = LSM6DSR_ERROR;
      break;
  }

  if (ret == LSM6DSR_ERROR)
  {
    return LSM6DSR_ERROR;
  }

  if (gyro_is_enabled == 1U)
  {
    ret = Set_G_ODR_When_Enabled(newOdr);
  }
  else
  {
    ret = Set_G_ODR_When_Disabled(newOdr);
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSR gyroscope sensor output data rate when enabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Set_G_ODR_When_Enabled(float Odr)
{
  lsm6dsr_odr_g_t new_odr;

  new_odr = (Odr <=   12.5f) ? LSM6DSR_GY_ODR_12Hz5
          : (Odr <=   26.0f) ? LSM6DSR_GY_ODR_26Hz
          : (Odr <=   52.0f) ? LSM6DSR_GY_ODR_52Hz
          : (Odr <=  104.0f) ? LSM6DSR_GY_ODR_104Hz
          : (Odr <=  208.0f) ? LSM6DSR_GY_ODR_208Hz
          : (Odr <=  417.0f) ? LSM6DSR_GY_ODR_417Hz
          : (Odr <=  833.0f) ? LSM6DSR_GY_ODR_833Hz
          : (Odr <= 1667.0f) ? LSM6DSR_GY_ODR_1667Hz
          : (Odr <= 3333.0f) ? LSM6DSR_GY_ODR_3333Hz
          :                    LSM6DSR_GY_ODR_6667Hz;

  /* Output data rate selection. */
  if (lsm6dsr_gy_data_rate_set(&reg_ctx, new_odr) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  return LSM6DSR_OK;
}

/**
 * @brief  Set the LSM6DSR gyroscope sensor output data rate when disabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Set_G_ODR_When_Disabled(float Odr)
{
  gyro_odr = (Odr <=   12.5f) ? LSM6DSR_GY_ODR_12Hz5
           : (Odr <=   26.0f) ? LSM6DSR_GY_ODR_26Hz
           : (Odr <=   52.0f) ? LSM6DSR_GY_ODR_52Hz
           : (Odr <=  104.0f) ? LSM6DSR_GY_ODR_104Hz
           : (Odr <=  208.0f) ? LSM6DSR_GY_ODR_208Hz
           : (Odr <=  417.0f) ? LSM6DSR_GY_ODR_417Hz
           : (Odr <=  833.0f) ? LSM6DSR_GY_ODR_833Hz
           : (Odr <= 1667.0f) ? LSM6DSR_GY_ODR_1667Hz
           : (Odr <= 3333.0f) ? LSM6DSR_GY_ODR_3333Hz
           :                    LSM6DSR_GY_ODR_6667Hz;

  return LSM6DSR_OK;
}


/**
 * @brief  Get the LSM6DSR gyroscope sensor full scale
 * @param  FullScale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Get_G_FS(int32_t  *FullScale)
{
  LSM6DSRStatusTypeDef ret = LSM6DSR_OK;
  lsm6dsr_fs_g_t fs_low_level;

  /* Read actual full scale selection from sensor. */
  if (lsm6dsr_gy_full_scale_get(&reg_ctx, &fs_low_level) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  switch (fs_low_level)
  {
    case LSM6DSR_125dps:
      *FullScale =  125;
      break;

    case LSM6DSR_250dps:
      *FullScale =  250;
      break;

    case LSM6DSR_500dps:
      *FullScale =  500;
      break;

    case LSM6DSR_1000dps:
      *FullScale = 1000;
      break;

    case LSM6DSR_2000dps:
      *FullScale = 2000;
      break;

    case LSM6DSR_4000dps:
      *FullScale = 4000;
      break;

    default:
      ret = LSM6DSR_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSR gyroscope sensor full scale
 * @param  FullScale the functional full scale to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Set_G_FS(int32_t FullScale)
{
  lsm6dsr_fs_g_t new_fs;

  new_fs = (FullScale <= 125)  ? LSM6DSR_125dps
         : (FullScale <= 250)  ? LSM6DSR_250dps
         : (FullScale <= 500)  ? LSM6DSR_500dps
         : (FullScale <= 1000) ? LSM6DSR_1000dps
         : (FullScale <= 2000) ? LSM6DSR_2000dps
         :                       LSM6DSR_4000dps;

  if (lsm6dsr_gy_full_scale_set(&reg_ctx, new_fs) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  return LSM6DSR_OK;
}

/**
 * @brief  Get the LSM6DSR gyroscope sensor raw axes
 * @param  Value pointer where the raw values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Get_G_AxesRaw(int16_t *Value)
{
  axis3bit16_t data_raw;

  /* Read raw data values. */
  if (lsm6dsr_angular_rate_raw_get(&reg_ctx, data_raw.u8bit) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  /* Format the data. */
  Value[0] = data_raw.i16bit[0];
  Value[1] = data_raw.i16bit[1];
  Value[2] = data_raw.i16bit[2];

  return LSM6DSR_OK;
}


/**
 * @brief  Get the LSM6DSR gyroscope sensor axes
 * @param  AngularRate pointer where the values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Get_G_Axes(int32_t *AngularRate)
{
  axis3bit16_t data_raw;
  float sensitivity;

  /* Read raw data values. */
  if (lsm6dsr_angular_rate_raw_get(&reg_ctx, data_raw.u8bit) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  /* Get LSM6DSR actual sensitivity. */
  if (Get_G_Sensitivity(&sensitivity) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  /* Calculate the data. */
  AngularRate[0] = (int32_t)((float)((float)data_raw.i16bit[0] * sensitivity));
  AngularRate[1] = (int32_t)((float)((float)data_raw.i16bit[1] * sensitivity));
  AngularRate[2] = (int32_t)((float)((float)data_raw.i16bit[2] * sensitivity));

  return LSM6DSR_OK;
}


/**
 * @brief  Get the LSM6DSR GYRO data ready bit value
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Get_G_DRDY_Status(uint8_t *Status)
{
  if (lsm6dsr_gy_flag_data_ready_get(&reg_ctx, Status) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  return LSM6DSR_OK;
}


/**
 * @brief  Get the LSM6DSR register value
 * @param  Reg address to be read
 * @param  Data pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Read_Reg(uint8_t Reg, uint8_t *Data)
{
  if (lsm6dsr_read_reg(&reg_ctx, Reg, Data, 1) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  return LSM6DSR_OK;
}


/**
 * @brief  Set the LSM6DSR register value
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSRStatusTypeDef LSM6DSRSensor::Write_Reg(uint8_t Reg, uint8_t Data)
{
  if (lsm6dsr_write_reg(&reg_ctx, Reg, &Data, 1) != LSM6DSR_OK)
  {
    return LSM6DSR_ERROR;
  }

  return LSM6DSR_OK;
}


int32_t LSM6DSR_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
  return ((LSM6DSRSensor *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}


int32_t LSM6DSR_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
  return ((LSM6DSRSensor *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}
