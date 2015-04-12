/*

  Contiki library for VCNL4000 proximity and light sensor - 
  For more details see http://www.vishay.com/product?docid=83798

  Author - Per Lindgren (per.o.lindgren@gmail.com)

  License - GPLv3

*/


#include <stdio.h>
#include "vcnl4000.h"
#include "i2cmaster.h"


// vcnl4000_exist(DEVADDR) checks on the i2c bus if the device is connected
// and has address DEVADDR. Returns 0 on success and 1 on failure.

uint8_t vcnl4000_exist(void)
{
  uint8_t result = 1;
  uint8_t ret;

  ret = i2c_start(VCNL4000_DEV_ADDR+I2C_WRITE) ;  // set device address and write mode
  if( !ret ){
    result = 0;
  }
  i2c_stop() ;

  return result;
}


// vcnl4000_version(&prod, &rev) returns in prod and rev the chips
// harware product id and revision id. As of this writing both values
// are 1. Any failure returns 1 for the function and leaves prod and rev
// untouched

uint8_t vcnl4000_version(uint8_t *prod, uint8_t *rev)
{
  uint8_t result = 1;
  uint8_t ret;

  ret = i2c_start(VCNL4000_DEV_ADDR+I2C_WRITE) ;  // set device address and write mode
  if(!ret) {
    ret = i2c_write(VCNL4000_PRODID_ADDR);
    if (!ret) {
      ret = i2c_rep_start(VCNL4000_DEV_ADDR+I2C_READ);
      if (!ret) {
	uint8_t val = i2c_readNak();
	// set return values
	if (val) {
	  *prod = val >> 4;
	  *rev = val & 0xF;
	  result = 0;
	}
      }
    }
  }
  i2c_stop() ;
  
  return result;
}

// vncl4000_get_aux is a helper function to get either ambient
// light value or proximity value. Both values are read in a
// similar way, so the difference can be parameterized.
// Function returns 0 on success and 1 otherwise.


uint8_t vcnl4000_aux_get(uint8_t cmd_bit,
			 uint8_t result_ready_mask,
			 uint8_t result_data_address,
			 uint16_t *return_value)
{
  uint8_t result = 1;
  uint16_t ret;
  
  ret = i2c_start(VCNL4000_DEV_ADDR+I2C_WRITE) ;  // set device address and read mode
  if(!ret) {
    i2c_write(VCNL4000_COMMAND);
    ret = i2c_write(cmd_bit);
    if (!ret) {
      // Wait for measurement to complete
      do {
	i2c_start(VCNL4000_DEV_ADDR+I2C_WRITE);
	i2c_write(VCNL4000_COMMAND);
	i2c_rep_start(VCNL4000_DEV_ADDR+I2C_READ);
	ret = i2c_readNak();
      } while (!(ret & result_ready_mask));

      // Now read result from "result_data_address"
      i2c_start_wait(VCNL4000_DEV_ADDR+I2C_WRITE);
      i2c_write(result_data_address);
      i2c_rep_start(VCNL4000_DEV_ADDR+I2C_READ);
      *return_value = i2c_readAck()<<8; // read first byte, high 8-bits
      *return_value += i2c_readNak(); // read second byte, low 8-bits
      result = 0;
    }
  }
  i2c_stop() ;
  
  return result;
}


// vncl4000_get_ambient_light(void) returns ambient light register 
// according to current parameters for measuring ambient light.
// Returns ambient light as unsigned 16-bit value in parameter value.
// Function returns 0 on success and 1 otherwise.

uint8_t vcnl4000_get_ambient(uint16_t *value)
{
  return vcnl4000_aux_get(VCNL4000_AMBIENT_SINGLE_ON_DEMAND_BIT,
			  VCNL4000_AMBIENT_DATA_READY_MASK,
			  VCNL4000_AMBIENT_HI_ADDR,
			  value);
}


// vncl4000_get_proximity(void) returns proximity register 
// according to current parameters for measuring proximity.
// Returns proximity as unsigned 16-bit value in parameter value.
// Function returns 0 on success and 1 otherwise.

uint8_t vcnl4000_get_proximity(uint16_t *value)
{
  return vcnl4000_aux_get(VCNL4000_PROXIMITY_SINGLE_ON_DEMAND_BIT, 
			  VCNL4000_PROXIMITY_DATA_READY_MASK,
			  VCNL4000_PROXIMITY_HI_ADDR, 
			  value);
}


// Get or set value of IR LED current multiplier register. Value is 6 bit, maximum value
// in rev 1.1 hardware is 20. Unit is 10 mA. Max value for hardware 1.1 is thus 200 mA.
// Parameters: pointer to uint8 to store (get) multiplier value, or the value to set 
// multiplier value to.
// Returns 1 on success, 0 on failure.

uint8_t vcnl4000_get_set_proximity_ir_led_current(uint8_t *value, uint8_t flag)
{
  uint8_t result = 0;
  uint8_t ret;

  if (flag == VCNL4000_PROXIMITY_GET_IR_LED_CURRENT) {
    ret = i2c_start(VCNL4000_DEV_ADDR+I2C_WRITE) ;  // set device address and read mode
    if(!ret) {
      i2c_write(VCNL4000_PROXIMITY_IR_LED_CURRENT);
      ret = i2c_rep_start(VCNL4000_DEV_ADDR+I2C_READ) ;  // set device address and read mode
      if(!ret) {
	ret = i2c_readNak();
	*value = ret & 0x3F;
	result = 1;
      }
    }
  } else if (flag == VCNL4000_PROXIMITY_SET_IR_LED_CURRENT) {
    ret = i2c_start(VCNL4000_DEV_ADDR+I2C_WRITE) ;  // set device address and read mode
    if(!ret) {
      ret = i2c_write(VCNL4000_PROXIMITY_IR_LED_CURRENT);
      ret = i2c_write(*value);
      if(!ret) {
	result = 1;
      }
    }
  }
  i2c_stop() ;
  
  return result;
}

// Get value of IR LED current multiplier register. Value is 6 bit, maximum value
// in rev 1.1 hardware is 20. Unit is 10 mA. Max value for hardware 1.1 is thus 200 mA.
// Calls helper routine to get the value. Returns 1 on success, 0 on failure.

uint8_t vcnl4000_get_proximity_ir_led_current(uint8_t *value)
{
  uint8_t result;

  result = vcnl4000_get_set_proximity_ir_led_current(value, VCNL4000_PROXIMITY_GET_IR_LED_CURRENT);

  return result;
}


// Set value of IR LED current multiplier register. Value is 6 bit, maximum value
// in rev 1.1 hardware is 20. Unit is 10 mA. Max value for hardware 1.1 is thus 200 mA.
// Calls helper routine to set the value. Returns 1 on success, 0 on failure.

uint8_t vcnl4000_set_proximity_ir_led_current(uint8_t value)
{
  uint8_t result;

  result = vcnl4000_get_set_proximity_ir_led_current(&value, VCNL4000_PROXIMITY_SET_IR_LED_CURRENT);

  return result;
}
