/*
  Contiki Rime BRD demo.
  Broadcast Temp, Bat Voltage, RSSI, LQI DEMO. Robert Olsson <robert@herjulf.se>

  Heavily based on code from:

 * Copyright (c) 2007, Swedish Institute of Computer Science.
 * All rights reserved.
 *
  See Contiki for full copyright.
*/

#include "bc.h"

#include "vcnl4000.h"
#include "ds18b20.h"

//#define SIZE          40

// get_eui64_addr() is declared in contiki-main.c
extern int get_eui64_addr(uint8_t *addr_array);

//unsigned char charbuf[SIZE];

unsigned int report_interval = DEFAULT_REPORT_INTERVAL;
int report_interval_changed = 0;


#define MAX_BCAST_SIZE 99

struct broadcast_message {
  uint8_t head; /* version << 4 + ttl */
  uint8_t seqno;
  uint8_t buf[MAX_BCAST_SIZE];  /* Check for max payload 20 extra to be able to test */
};

// Max TTL for relayd messages is 0xF
#define DEF_TTL 0xF
uint8_t  ttl = DEF_TTL;

#define DEF_CHAN 26


// Struct for collecting statistics about messages

struct {
  uint16_t sent;
  uint16_t dup;
  uint16_t ignored;
  uint16_t ttl_zero;
} relay_stats;


// Keep track of radio neighbors in a list, neighbors_list

// The ->last_rssi and ->last_lqi fields hold the Received Signal
// Strength Indicator (RSSI) and CC2420 Link Quality Indicator (LQI)
// values that are received for the incoming broadcast packets. */
// The ->avg_gap contains the average seqno gap that we have seen
// from this neighbor. */

struct neighbor {
  struct neighbor *next;
  rimeaddr_t addr;
  uint16_t last_rssi, last_lqi;
  uint8_t last_seqno;
  uint32_t avg_seqno_gap;
};

#define MAX_NEIGHBORS 64

MEMB(neighbors_memb, struct neighbor, MAX_NEIGHBORS);
LIST(neighbors_list);

static struct broadcast_conn broadcast;

#define SEQNO_EWMA_UNITY 0x100 /* Moving average */
#define SEQNO_EWMA_ALPHA 0x040


// 
//

PROCESS(init_process, "Init process");
PROCESS(broadcast_process, "Broadcast process");

/* The Shell commands initialised here
 * 1. The Report Interval Command rint <n> where n represents periodicity in seconds 
 * 2. The Report Mask command rmask <n> where n represents the 8 bit mask for data fields
 *    in the broadcast report   
*/

#define SHELL_INTERFACE

#ifdef SHELL_INTERFACE
PROCESS(ri_process , "Set Report Interval");
SHELL_COMMAND( ri_command, 
               "ri", 
               "ri <number> [Set interval between reports to \"number\" seconds]", 
               &ri_process) ;

PROCESS(mask_process , "Set Report Mask");
SHELL_COMMAND( mask_command, 
               "mask", 
               "mask [ [+|-]field-name] | field-name ] [Add or remove data fields in the Sensor Report. No arguments lists current fields]",
               &mask_process) ;

PROCESS(bootloader_process , "Reboot node into boot loader");
SHELL_COMMAND( bootloader_command, 
               "bl", 
               "bl [Reboot node into boot loader]", 
               &bootloader_process) ;
#endif

/* -----------------------------------------------------------------------------------------------------------------*/


//PROCESS(test_serial, "Serial line test process");
//AUTOSTART_PROCESSES( &init_process,&test_serial, &broadcast_process);
AUTOSTART_PROCESSES( &init_process, &broadcast_process);


// Turn the red or yellow LEDs on or off                                                                                   
#define LED_ON 1
#define LED_OFF 0

void set_led(uint8_t LED, uint8_t onoff)
{
  // Set output mode for pin                                                                                                    
  DDRE |= (1<<LED);
  if (onoff)
    PORTE &= ~(1<<LED); /* Turn led on */
  else
    PORTE |= 1<<LED; /* Turn led off */
}


#if 0
void print_help_command_menu()
{
  printf("\n------------------Command Menu--------------------------------\n") ;
  printf("\nh\tPrints a list of supported commands\n\tUsage: h\n") ;
  printf("ri\tSets the report interval\n\tUsage: ri <period in seconds>\n");
  printf("rm\tSets the report mask\n\tUsage: rm <8 bit value representing the mask>\n");
  printf("Mask bits correspond to the following tags MSB -> LSB:\n");
  printf("| UT | PS | T | T_MCU | V_MCU | UP | V_IN | spare bit |\n\n") ;
}


/*
We have a set of tags that identify the data items in each report.
Tagmask is an 8 bit character whose bits are mapped to the report tags
and by setting or resetting these bits the report size can be manipuated

         MSB                                          LSB
   bits:  7    6    5     4      3      2       1      0
        ---------------------------------------------------
        | UT | PS | T | T_MCU | V_MCU | UP | V_IN | spare |
        ---------------------------------------------------

    VALID VALUES:
        0xFE which enables all these tags.
        A report set by 0xFE
	2012-05-22 14:07:46 UT=1337688466 ID=283c0cdd030000d7 PS=0 T=30.56  T_MCU=34.6  V_MCU=3.08 UP=2C15C V_IN=4.66 
*/

// Union with 8-bit int and Bitwise structure for mapping the input mask
// The int is for easy bulk setting/getting of the flags

// TODO 2014-08-25 (PerL): Change "tagmask" to 16 bits to support more sensors, and
// use it for DS18B20 temp sensor, proximity sensor, and so on...

union {
  uint8_t all_reportmask_flags;
  struct tag_mask {
    unsigned ut:1 ;
    unsigned ps:1 ;
    unsigned t:1 ;
    unsigned t_mcu:1 ;
    unsigned v_mcu:1 ;
    unsigned up:1 ;
    unsigned v_in:1 ;
    unsigned spare:1 ;    
  } reportmask ;
} reportmask_un;
#endif

unsigned char eui64_addr[8] ;

#if 0
PROCESS_THREAD(test_serial, ev, data)
{
   PROCESS_BEGIN();
   
   const char *delimiter = " ";
   char *command = NULL ;
   char *temp = NULL ;
   //   unsigned char  ri = 0 ; 
   
   printf("Process begins for test_serial !! \n") ;
   
   for(;;) {
     PROCESS_YIELD_UNTIL(ev == serial_line_event_message);
     
          printf("serial: %s\n", (char*)data);
     
     command = (char*)strsep((char**)&data, delimiter) ;		
     
     if (!strncmp(command, "h",1)) {
       print_help_command_menu() ;
     } else if(!strncmp(command,"ri",2)) {
       temp = strsep((char**)&data, delimiter) ;
       if (temp == NULL)
	 printf("Report interval = %d\n", report_interval);
       else {
	 report_interval = atoi(temp);
	 if (report_interval < 0) /* can't have a negative report_interval */
	   report_interval = -report_interval;
       } 
       printf("New report interval = %d\n", report_interval);
     } else if (!strncmp(command, "rm",7)) {
       temp = strsep((char**)&data, delimiter);
       if (temp == NULL)
	 printf("Report mask = 0x%x\n", reportmask_un.all_reportmask_flags);
       else {
	 unsigned char mask = atoi(temp);
	 printf("New report mask = 0x%x\n", mask);
	 reportmask_un.all_reportmask_flags = mask;
       }
     } else if (command[0] == 0) {
       printf("<null> command\n") ;      
     } else 
       printf("%s: Invalid command. Try h for a list of commands\n", command) ;
   }	// more commands can be added here

   PROCESS_END();
}
#endif


/*
 * Initialize real time clock, to generate ISR every 1 second
 */

static void rtcc_init(void)
{
  TIMSK2 &= ~((1<<TOIE2) | (1<<OCIE2A));  /* Disable TC0 interrupt */
  ASSR |= (1<<AS2); /* set Timer/Counter2 to be asynchronous */
  TCNT2 = 0x00;
  TCCR2B = 0x05;  /* prescale the timer to / 128 to --> 1s */
  //TCCR2B = 0x02;  /* prescale the timer to / 8   16 Hz*/
  while (ASSR & 0x07)
    ;
  TIMSK2 |= (1<<TOIE2);  
}


ISR(TIMER2_OVF_vect)
{ 
  static int rtc = DEFAULT_REPORT_INTERVAL;
  
  set_led(LED_RED, LED_OFF);
  set_led(LED_YELLOW, LED_OFF);
  
  if (report_interval_changed) {
    rtc = report_interval;
    report_interval_changed = 0;
  }
  
  if (rtc >= 0 && --rtc == 0) {
    rtc = report_interval;
    process_post(&broadcast_process, 0x12, NULL);
  }
}


void read_sensors(void)
{
  uint16_t h;
  uint8_t p1;
  
  BATMON = 16; //Stabilize at highest range and lowest voltage

  ADCSRB |= (1<<MUX5);   //this bit buffered till ADMUX written to!
  ADMUX = 0xc9;         // Select internal 1.6 volt ref, temperature sensor ADC channel
  ADCSRA = 0x85;        //Enable ADC, not free running, interrupt disabled, clock divider 32 (250 KHz@ 8 MHz)
  ADCSRA |= (1<<ADSC);         //Throwaway conversion

  while (ADCSRA & (1<<ADSC))
    ; //Wait
  ADCSRA |= (1<<ADSC);          //Start conversion
  
  while (ADCSRA & (1<<ADSC))
    ; //Wait
  
  h = ADC;
  param_T_MCU = (double) h * 1.13 - 272.8;

  //  h = 11*h-2728+(h>>2);       //Convert to celcius*10 (should be 11.3*h, approximate with 11.25*h)
  ADCSRA = 0;               //disable ADC
  ADMUX = 0;                //turn off internal vref      

  //  m = h/10; s=h-10*m;

  for ( p1=16; p1<31; p1++) { 
    BATMON = p1;
    /*  delay_us(100); */
    if ((BATMON & (1<<BATMON_OK)) == 0 ) 
      break;
  }
  param_V_MCU = (double) 2550-75*16-75 + 75 * p1; //-75 to take the floor of the 75 mv transition window
  param_V_MCU = param_V_MCU/1000;

  if (vcnl4000_exist() == 0) {
    // ambient_light value, if sensor is present
    vcnl4000_get_ambient(&param_ambient_light);
    //proximity value, if sensor is present
    vcnl4000_get_proximity(&param_proximity);
  }

  param_T = -275.0;
  if (ds18b20_probe()) {
    int numtries = 10;
    uint8_t res = 0;
    while (numtries-- && !res) {
      res = ds18b20_get_temp(&param_T);
      //     if (res == 1 && numtries < 9) ;
      //	printf("ds18b20_get_temp() on retry!\n");
    };
  } else
    printf("ds18b20_probe() failed.\n");
    
}

static void broadcast_recv(struct broadcast_conn *c, const rimeaddr_t *from)
{
  struct neighbor *n;
  struct broadcast_message *msg;
  uint8_t seqno_gap;
  //uint8_t relay = 1;   /* Default on */

  set_led(LED_RED, LED_ON);

  msg = packetbuf_dataptr();

  if((msg->head >> 4) != 1 || (msg->head & 0xF) == 0) {
    //relay = 0;
    relay_stats.ignored++;
    goto out;
  }

  // From our own address. Can happen if we receive own pkt via relay
  // Ignore the message in that case

  if(rimeaddr_cmp(&rimeaddr_node_addr, from)) {
    //relay = 0;
    relay_stats.ignored++;
    goto out;
  }

  // Check if we already know this neighbor. 
  for (n = list_head(neighbors_list); n != NULL; n = list_item_next(n))
    if (rimeaddr_cmp(&n->addr, from)) 
      break;
  
  // If n is NULL we did not find "from" in the neighbor list

  if (n == NULL) {
    n = memb_alloc(&neighbors_memb); /* New neighbor */
    
    // Did memory allocation fail?
    if ( !n ) 
      goto out;
    
    rimeaddr_copy(&n->addr, from);
    n->last_seqno = msg->seqno - 1;
    n->avg_seqno_gap = SEQNO_EWMA_UNITY;
    list_add(neighbors_list, n);
  }

  n->last_rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
  n->last_lqi = packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY);

  /* Compute the average sequence number gap from this neighbor. */
  seqno_gap = msg->seqno - n->last_seqno;
  
  n->avg_seqno_gap = (((uint32_t)seqno_gap * SEQNO_EWMA_UNITY) *
                      SEQNO_EWMA_ALPHA) / SEQNO_EWMA_UNITY +
                      ((uint32_t)n->avg_seqno_gap * (SEQNO_EWMA_UNITY -
                                                     SEQNO_EWMA_ALPHA)) / SEQNO_EWMA_UNITY;

  n->last_seqno = msg->seqno;

  printf("&: %s [ADDR=%d.%d SEQ=%d TTL=%u RSSI=%u LQI=%u DRP=%d.%02d]\n",
	 msg->buf,
	 from->u8[0], from->u8[1], msg->seqno, msg->head & 0xF,
	 n->last_rssi,
	 n->last_lqi, 
	 (int)(n->avg_seqno_gap / SEQNO_EWMA_UNITY),
	 (int)(((100UL * n->avg_seqno_gap) / SEQNO_EWMA_UNITY) % 100));

out:
  set_led(LED_RED, LED_OFF);

}


static const struct broadcast_callbacks broadcast_call = {broadcast_recv};

/*
  Channel 11 - 26 

  MAX TX_PWR_3DBM       ( 0 )
  MIN TX_PWR_17_2DBM    ( 15 )

  Channels (11, 15, 20, 25 are WiFi-free).
  RSSI is high while LQI is low -> Strong interference, 
  Both are low -> Try different locations.
*/


// Uitlity funktion to add strings together, if dst string has enough room.
// Max size of dst is constant MAX_BCAST_SIZE
uint8_t add_strings(char* src, uint8_t *dst)
{
  if ((strlen(src) + strlen((char*)dst)) < MAX_BCAST_SIZE) {
    strcat((char*)dst, src);
  }
  return strlen((char*)dst);
}

// Size of largest individual sensor value-pair string. Increase if necessary
// when adding support for new sensor values
#define MAX_SENSOR_VALUE_PAIR_STRING_LENGTH 32

PROCESS_THREAD(broadcast_process, ev, data)
{

  static uint8_t seqno;
  struct broadcast_message msg;
  char strbuf[MAX_SENSOR_VALUE_PAIR_STRING_LENGTH]; 
  uint8_t ch, txpwr;

  PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
  PROCESS_BEGIN();

  broadcast_open(&broadcast, 129, &broadcast_call);

  //rf230_set_txpower(15);
  txpwr = rf230_get_txpower();
  //ch = rf230_get_channel();
  ch = DEF_CHAN;
  rf230_set_channel(ch);
  printf("Ch=%-d TxPwr=%-d\n", ch, txpwr);

  while (1) {
    int len, acc_len; 

    PROCESS_YIELD_UNTIL(ev == 0x12);

    set_led(LED_YELLOW, LED_ON);

    // All values are calculated in the read_sensors() routine

    read_sensors();

    // Compose log string, depending on the current "mask". One if-statement for each
    // possible type of value
	
    len = 0; // accumulated length of log string
    acc_len = 0;
    msg.buf[0] = 0;

    if (sensor_info_array[E64].show_value_flag == 1) {
      // write the EUI64 MAC address to the message
      len = sprintf(strbuf, " E64=%02x%02x%02x%02x%02x%02x%02x%02x", 
		    eui64_addr[0],eui64_addr[1], eui64_addr[2], eui64_addr[3], 
		    eui64_addr[4], eui64_addr[5], eui64_addr[6], eui64_addr[7]) ;
      // Check to se if value-pair fits inside msg.buf
      acc_len = add_strings(strbuf, msg.buf);
    }

    if (sensor_info_array[T].show_value_flag == 1 && acc_len < MAX_BCAST_SIZE) {
      if (param_T > -275.0) {
	len += sprintf(strbuf, " T=%.2f", (double)param_T);
	// Check to se if value-pair fits inside msg.buf
	acc_len = add_strings(strbuf, msg.buf);
      }
    }

    if (sensor_info_array[V_MCU].show_value_flag == 1 && acc_len < MAX_BCAST_SIZE) {
      // write voltage to the microprocessor to the message
      len += sprintf(strbuf, " V_MCU=%-.2f", param_V_MCU);
      // Check to se if value-pair fits inside msg.buf
      acc_len = add_strings(strbuf, msg.buf);
    }

    if (sensor_info_array[T_MCU].show_value_flag == 1 && acc_len < MAX_BCAST_SIZE) {
      // write temperature och the microprocessor unit to the message
      len += sprintf(strbuf, " T_MCU=%-.2f", param_T_MCU);
      // Check to se if value-pair fits inside msg.buf
      acc_len = add_strings(strbuf, msg.buf);
    }

    if (sensor_info_array[PROX].show_value_flag == 1 && acc_len < MAX_BCAST_SIZE) {
      len += sprintf(strbuf, " PROX=%u", param_proximity);
      // Check to se if value-pair fits inside msg.buf
      acc_len = add_strings(strbuf, msg.buf);
    }

    if (sensor_info_array[AMB].show_value_flag == 1 && acc_len < MAX_BCAST_SIZE) {
      len += sprintf(strbuf, " AMB=%u", param_ambient_light);
      // Check to se if value-pair fits inside msg.buf
      acc_len = add_strings(strbuf, msg.buf);
    }

    packetbuf_copyfrom(&msg, acc_len+1);

    msg.head = 1<<4; /* Version 1 */
    msg.head |= DEF_TTL;
    msg.seqno = seqno++; // May wrap! But that's OK...
    
    broadcast_send(&broadcast);
    
    printf("&:%s\n", msg.buf);

    set_led(LED_YELLOW, LED_OFF);
  }

  PROCESS_END(); // never reached!
}


PROCESS_THREAD(init_process, ev, data)
{
  PROCESS_BEGIN();

  printf("Init\n");

  set_led(LED_RED, LED_OFF);
  set_led(LED_YELLOW, LED_OFF);

  // Set pin DDD4 as output
  DDRD |= (1<<4);

  // Set ISR to one second
  rtcc_init();

  // Registering the shell commands
  serial_line_init();
  serial_shell_init();
  shell_register_command(&ri_command);
  shell_register_command(&mask_command);
  shell_register_command(&bootloader_command);

  // Get the EUI64 id 
  (void)get_eui64_addr((uint8_t *)eui64_addr) ;
  //  memset(&reportmask_un.reportmask,0x18, 1) ; // enable t_mcu, v_mcu tags
   
  PROCESS_END();
}


// Set report interval in seconds. Report interval of zero (0) turns reporting off. 
// Interval an be 5 digits or less

PROCESS_THREAD(ri_process, ev, data)
{
  PROCESS_BEGIN();
  
  if (data == NULL || strlen((char*)data) == 0) {
    // Without an argument, print current value of report interval
    printf("ri=%d\n", report_interval);
  } else {
    int result = 0;
    char arg_str[6]; // Room for five digits and terminating NUL
    arg_str[0] = 0;
    
    if (data != NULL && strlen((char*)data) < 6) {
      result = sscanf((const char*)data, "%[0-9]", arg_str);
    }

    if (result == 1) {
      // Here we KNOW that arg_str is made of digits and maximum 5 digits long
      report_interval = atoi((const char*)arg_str);
      report_interval_changed = 1;
    }

  }

  PROCESS_END() ; 
}


// "mask T" and "mask +T" is equal, meaning show T value. Opposite is "mask -T".

PROCESS_THREAD(mask_process, ev, data)
{
  PROCESS_BEGIN();

  int i;
  
  if (data == NULL || strlen((char*)data) == 0) {
    int print_newline = 0;

    for (i = 0; ; i++) {
      sensor_info_data_struct sensor_info = sensor_info_array[i];
      if (sensor_info.name == NULL)
      	break;
      else {
	if (sensor_info.show_value_flag == 1) {
	  printf("%s ", sensor_info.name);
	  print_newline = 1;
	}
      }
    }

    if (print_newline)
      printf("\n");
  } else {
    //    printf("Shell Command \"mask %s\" invoked\n", (char*)data);
    char data_copy[64];
    char *sep = " ";
    char *word;
    int set_show_flag_to; 

    data_copy[63] = 0;
    (void)strncpy(data_copy, data, 63);
    
    for (word = strtok(data_copy, sep); word; word = strtok(NULL, sep)) {
      set_show_flag_to = 1 ;
      if (word[0] == '+')
	word++;
      if (word[0] == '-') {
	word++;
	set_show_flag_to = 0 ;
      }

      for (i = 0; ; i++) {
	sensor_info_data_struct si = sensor_info_array[i];
	
	if (si.name == NULL)
	  break;
	
	if (strcmp(si.name, word) == 0) {
	  //  printf("matched name!\n");
	  sensor_info_array[i].show_value_flag = set_show_flag_to;
	}
      }
    }
  }
  
  PROCESS_END() ;
}


// Command "bl" for resetting node and going into boot loader

PROCESS_THREAD(bootloader_process, ev, data)
{
  PROCESS_BEGIN();
  
  asm("jmp 0xf800");

  PROCESS_END() ; 
}


static struct pt send_thread;

PT_THREAD(send_message(char *txt, rimeaddr_t b, int *retval))
{
  PT_BEGIN(&send_thread);

    printf("%s \n", txt);

  *retval = 0;
  PT_END(&send_thread);
}
