/*
 * i2c_slave.c
 *
 *  Created on: Jul 31, 2017
 *      Author: epanda
 */

#include <stdint.h>
#include <stdlib.h>
#include "i2c_slave.h"


#define gpio_w32(b, off)  do { } while (0)
#define gpio_r32(off)   do { } while (0)

#define I2C_STATE_NACK     1
#define I2C_STATE_ACK      0
#define I2C_STATE_IDLE    -1
#define I2C_STATE_START   -2
#define I2C_STATE_STOP    -3
#define I2C_STATE_DEVICE  -4
#define I2C_STATE_TIMEOUT -5

#define I2C_RET_OK     0
#define I2C_RET_END    -1

#define I2C_DEV_OFFS  0x0
#define I2C_DEV_DATA  0x1

static inline int32_t slave_data_receive(struct i2c_slave *slave);
static inline int32_t slave_data_send(struct i2c_slave *slave);
static inline int32_t slave_byte_read(struct i2c_slave *slave);
static inline int32_t slave_byte_write(struct i2c_slave *slave, uint8_t val);
static inline int32_t i2c_dev_address_check(struct i2c_slave *slave, int32_t val);
static inline void i2c_ack_send(struct i2c_slave *slave);
static inline int32_t i2c_ack_read(struct i2c_slave *slave);
static inline void i2c_pins_init(struct i2c_slave *slave);
static inline int32_t i2c_scl_get(struct i2c_slave *slave);
static inline int32_t i2c_sda_get(struct i2c_slave *slave);
static inline void i2c_sda_set(struct i2c_slave *slave, int32_t val);
static inline int32_t wait_for_scl(struct i2c_slave *slave, int32_t level);
static inline int32_t gpio_value_get(uint32_t gpio);
static inline void gpio_value_set(uint32_t gpio, int32_t val);
static inline void gpio_direction_input(uint32_t gpio);
static inline void gpio_direction_output(uint32_t gpio, int32_t val);
static inline void delay(uint32_t count);

int32_t i2c_slave_init(void)
{
  uint32_t i, j;
  struct i2c_slave *slave;
  uint8_t slave_addr[I2C_SLAVE_DEV_MAX] =
  {
    /* 7 bits address */
    0xA0>>0x1, 0xB0>>0x1, 0xC0>>0x1, 0xD0>>0x1, 0xE0>>0x1
  };

  slave = malloc(sizeof(struct i2c_slave));
  if(!slave)
  {
    return -1;
  }

  for(j = 0; j < I2C_SLAVE_DEV_MAX; j++)
  {
    slave->dev[j].data_offs = 0;
    slave->dev[j].addr = slave_addr[j];
    for(i = 0; i < sizeof(slave->dev[0].data); i++)
    {
      slave->dev[j].data[i] = 0;
    }
  }

  i2c_pins_init(slave);
  return 0;
}

int32_t i2c_slave_cleanup(struct i2c_slave *slave)
{
  free(slave);
  slave = NULL;
  return 0;
}

int32_t i2c_slave_sda_interrupt_callback(struct i2c_slave *slave)
{
  int32_t val;
  uint32_t idx;

  if(wait_for_scl(slave, 0) == I2C_RET_END)
  {
    /* timeout */
    goto end;
  }

  slave->state = I2C_STATE_START;

  while(slave->state == I2C_STATE_START)
  {
    /* read address + R/W bit */
    val = slave_byte_read(slave);
    if(val == I2C_RET_END)
    {
      if(slave->state == I2C_STATE_START)
      {
        continue;
      }
      /* timeout, stop */
      goto end;
    }

    idx = i2c_dev_address_check(slave, val);
    if(idx == I2C_RET_END)
    {
      /* device address mismatch */
      goto end;
    }

    slave->state = I2C_STATE_DEVICE;

    /* send ACK */
    i2c_ack_send(slave);

    slave->dev_idx = idx;

    /* check R/W bit */
    if(val & 1)
    {
      //TODO: bug.master can read data without pre-send device data offset
      val = slave_data_send(slave);
      if(val == I2C_RET_END)
      {
        if(slave->state == I2C_STATE_START)
        {
          continue;
        }
        else
        {
          goto end;
        }
      }
    }
    else
    {
      val = slave_data_receive(slave);
      if(val == I2C_RET_END)
      {
        if(slave->state == I2C_STATE_START)
        {
          continue;
        }
        else
        {
          goto end;
        }
      }
    }
  }

  /* wait scl and sda high */
  while (!(i2c_scl_get(slave) && i2c_sda_get(slave)));
  slave->state = I2C_STATE_IDLE;
  return 0;

end:
  i2c_pins_init(slave);
  slave->state = I2C_STATE_IDLE;
  return -1;
}

static inline int32_t i2c_scl_get(struct i2c_slave *slave)
{
  int32_t val = gpio_value_get(slave->scl);
  return val;
}

static inline void i2c_sda_set(struct i2c_slave *slave, int32_t val)
{
  if (val)
    gpio_direction_input(slave->sda);
  else
    gpio_direction_output(slave->sda, 0);
}

static inline int32_t i2c_sda_get(struct i2c_slave *slave)
{
  int32_t val = gpio_value_get(slave->sda);
  return val;
}

static inline void i2c_pins_init(struct i2c_slave *slave)
{
  gpio_direction_input(slave->scl);
  gpio_direction_input(slave->sda);
}

static inline void gpio_direction_input(uint32_t gpio)
{
  (void)gpio;
}

static inline void gpio_direction_output(uint32_t gpio, int val)
{
  gpio_value_set(gpio, val);
}

static inline int32_t gpio_value_get(uint32_t gpio)
{
  return 1;
}

static inline void gpio_value_set(uint32_t gpio, int32_t val)
{
  (void)gpio;
  (void)val;
}

static void i2c_slave_store_data(struct i2c_slave *slave, uint8_t flag, uint8_t data)
{
  switch(flag)
  {
    case I2C_DEV_OFFS:
      slave->dev[slave->dev_idx].data_offs = data;
      break;

    case I2C_DEV_DATA:
      slave->dev[slave->dev_idx].data[slave->dev[slave->dev_idx].data_offs] = data;
      slave->dev[slave->dev_idx].data_offs++;
      break;

    default:
      break;
  }
}

static inline int32_t i2c_dev_address_check(struct i2c_slave *slave, int32_t val)
{
  uint32_t i;
  for(i = 0; i < I2C_SLAVE_DEV_MAX; i++)
  {
    if(slave->dev[i].addr == (val>>1))
      return i;
  }
  return -1;
}

static inline void delay(uint32_t count)
{
  while (--count > 0);
}

static inline int32_t wait_for_scl(struct i2c_slave *slave, int32_t level)
{
  uint32_t i = 1000;
  while(i2c_scl_get(slave) != level)
  {
    if(--i == 0)
    {
      slave->state = I2C_STATE_TIMEOUT;
      return I2C_RET_END;
    }
    delay(1);
  }
  return I2C_RET_OK;
}

static inline int32_t slave_byte_write(struct i2c_slave *slave, uint8_t val)
{
  uint8_t i;

  for(i = 0; i < 8; i++)
  {
    i2c_sda_set(slave, val & (1 << (7-i)));
    if(wait_for_scl(slave, 1) == I2C_RET_END)
    {
      return I2C_RET_END;
    }
    if(wait_for_scl(slave, 0) == I2C_RET_END)
    {
      return I2C_RET_END;
    }
  }
  i2c_sda_set(slave, 1);
  return I2C_RET_OK;
}

static inline int32_t slave_byte_read(struct i2c_slave *slave)
{
  uint8_t i;
  int32_t val, temp;

  for(i = 0x0; i < 0x8; i++)
  {
    while(!i2c_scl_get(slave))
    {
      //TODO:timeout check
    }

    val = (val<<0x1) | i2c_sda_get(slave);
    while(i2c_scl_get(slave))
    {
      //TODO:timeout check
      /* sda is drivered by master now.
       * if it changes when scl is high,stop or start happened */
      temp = i2c_sda_get(slave);
      if(!i2c_scl_get(slave))
        break;
      if((val & 1) != temp)
      {
        if(temp)
        {
          slave->state = I2C_STATE_STOP;
        }
        else
        {
          slave->state = I2C_STATE_START;
        }
        return I2C_RET_END;
      }
    }
  }
  return I2C_RET_OK;
}

static inline void i2c_ack_send(struct i2c_slave *slave)
{
  /* slave driver sda to low for ACK */
  i2c_sda_set(slave, 0);
  /* wait master read(scl rising edge trigger) ACK */
  while(!i2c_scl_get(slave))
  {
    //TODO:timeout check
  }

  /* wait scl to low */
  while(i2c_scl_get(slave))
  {
    //TODO:timeout check
  }
  /* slave release sda */
  i2c_sda_set(slave, 1);
}

static inline int32_t i2c_ack_read(struct i2c_slave *slave)
{
  int32_t val, temp;

  /* wait master set sda */
  while(!i2c_scl_get(slave))
  {
    //TODO:timeout check
  }
  /* read ACK */
  val = i2c_sda_get(slave);
  /* keep checking sda when scl high */
  while(i2c_scl_get(slave))
  {
    //TODO:timeout check
    /* sda is drivered by master now.
     * if it changes when scl is high,stop or start happened */
    temp = i2c_sda_get(slave);
    if(!i2c_scl_get(slave))
      break;

    if(val != temp)
    {
      if(temp)
      {
        slave->state = I2C_STATE_STOP;
      }
      else
      {
        slave->state = I2C_STATE_START;
      }
      return I2C_RET_END;
    }
  }

  if(val == 0x0)
  {
    /* ACK */
    return I2C_RET_OK;
  }
  else
  {
    /* NACK */
    return I2C_RET_END;
  }
}

static inline int32_t slave_data_send(struct i2c_slave *slave)
{
  uint8_t val, offs;

  offs = slave->dev[slave->dev_idx].data_offs;
  do
  {
    val = slave->dev[slave->dev_idx].data[offs];
    offs ++;
    if (slave_byte_write(slave, val) == I2C_RET_END)
    {
      return I2C_RET_END;
    }
  } while (i2c_ack_read(slave) == I2C_RET_OK);

  slave->dev[slave->dev_idx].data_offs = 0;
  slave->state = I2C_STATE_IDLE;
  return I2C_RET_OK;
}

static inline int32_t slave_data_receive(struct i2c_slave *slave)
{
  int32_t val = 0;
  uint8_t flag = I2C_DEV_OFFS;

  do
  {
    val = slave_byte_read(slave);
    if (val == I2C_RET_END)
    {
      return I2C_RET_END;
    }

    i2c_ack_send(slave);
    i2c_slave_store_data(slave, flag, val);
    flag = I2C_DEV_DATA;
  } while (val != I2C_RET_END);

  return I2C_RET_OK;
}
