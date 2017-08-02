/*
 * i2c_slave.h
 *
 *  Created on: Jul 31, 2017
 *      Author: epanda
 */

#ifndef I2C_SLAVE_H_
#define I2C_SLAVE_H_

#define I2C_SLAVE_DEV_MAX  5

struct i2c_device
{
  uint8_t addr;
  uint8_t data_offs;
  uint8_t data[256];

  /* something such as data field
   * changed flag, write protect.ect.. */
};

struct i2c_slave
{
  uint32_t scl;
  uint32_t sda;
  int32_t state;

  int32_t dev_idx;
  struct i2c_device dev[I2C_SLAVE_DEV_MAX];
};

#endif /* I2C_SLAVE_H_ */
