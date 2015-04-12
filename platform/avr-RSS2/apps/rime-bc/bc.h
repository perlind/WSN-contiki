/*
  Contiki Rime BRD demo.
  Broadcast Temp, Bat Voltage, RSSI, LQI DEMO. Robert Olsson <robert@herjulf.se>
  Further modifications by Per Lindgren <per.o.lindgren@gmail.com>

  Heavily based on code from:

 * Copyright (c) 2007, Swedish Institute of Computer Science.
 * All rights reserved.
 *
  See Contiki for full copyright.
*/

#ifndef __bc_h__
#define __bc_h__


#include <avr/sleep.h>
#include "contiki.h"
#include "contiki-net.h"
#include "lib/list.h"
#include "lib/memb.h"
#include "lib/random.h"
#include "net/rime.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//#include <time.h>
#include "rss2.h"
#include <avr/io.h>
#include "rf230bb.h"
#include "shell.h"
#include "serial-shell.h"
#include "dev/serial-line.h"


// Time between reports. Default is DEFAULT_REPORT_INTERVAL seconds. Value of 0 means no reports. Maximum value is ???

#define DEFAULT_REPORT_INTERVAL 10
extern unsigned int report_interval;


// Variables for different sensors/functions

enum SENSOR_TYPES {
  version = 1,
  E64 = 0,
  T,
  V_MCU,
  T_MCU,
  PROX,
  AMB,
  sizeof_sensortype_enum
};

typedef struct SENSOR_INFO {
  enum SENSOR_TYPES sensor_type;
  char* name;
  uint8_t show_value_flag;
} sensor_info_data_struct;

// Load an array, in the same ordes as the enum types, with "printname" and display flag
sensor_info_data_struct sensor_info_array[] = {
  { E64, "E64", 1 },
  { T, "T", 1 },
  { V_MCU, "V_MCU", 1 },
  { T_MCU, "T_MCU", 1 },
  { PROX, "PROX", 1 },
  { AMB, "AMB", 1 },
  { -1, NULL, 0 }
};

/* Processor input voltage */
double param_V_MCU; // voltage of XXX (?)

/* Processor temperature */
double param_T_MCU; // processor temp sensor, parameter T_MCU

/* From VCNL4000 we get proximity reading and ambient light reading */
uint16_t param_proximity;
uint16_t param_ambient_light;

/* Onboard DS18B20 temp sensor */
float param_T; 


#endif
