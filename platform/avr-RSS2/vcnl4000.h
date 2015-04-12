/*

  Contiki library for VCNL4000 proximity and light sensor - 
  For more details see http://www.vishay.com/product?docid=83798

  Author - Per Lindgren (per.o.lindgren@gmail.com)

  License - GPLv3

*/


#ifndef  __VCNL4000_H__
#define __VCNL4000_H__

#include "inttypes.h"

#define VCNL4000_DEV_ADDR 0x26
#define VCNL4000_COMMAND 0x80
#define VCNL4000_PRODID_ADDR 0x81
#define VCNL4000_AMBIENT_SINGLE_ON_DEMAND_BIT 0x10
#define VCNL4000_AMBIENT_HI_ADDR 0x85
#define VCNL4000_AMBIENT_LO_ADDR 0x86
#define VCNL4000_AMBIENT_DATA_READY_MASK 0x40
#define VCNL4000_PROXIMITY_SINGLE_ON_DEMAND_BIT 0x8
#define VCNL4000_PROXIMITY_HI_ADDR 0x87
#define VCNL4000_PROXIMITY_LO_ADDR 0x88
#define VCNL4000_PROXIMITY_DATA_READY_MASK 0x20
#define VCNL4000_PROXIMITY_IR_LED_CURRENT 0x83
#define VCNL4000_PROXIMITY_SET_IR_LED_CURRENT 0x1
#define VCNL4000_PROXIMITY_GET_IR_LED_CURRENT 0x2




// vcnl4000_exist() checks on the i2c bus if the device is connected.
// Returns 0 on success and 1 on failure.

extern uint8_t vcnl4000_exist(void);
extern uint8_t vcnl4000_version(uint8_t *prod, uint8_t *rev);
extern uint8_t vcnl4000_get_ambient(uint16_t *value);
extern uint8_t vcnl4000_get_proximity(uint16_t *value);
extern uint8_t vcnl4000_get_proximity_ir_led_current(uint8_t *value);
extern uint8_t vcnl4000_set_proximity_ir_led_current(uint8_t value);

#endif
