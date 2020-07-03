/********************************************************************
* Description:  hal_oshwdrv.c
*               HAL driver for the "OsHwDrv" board.
*               See http://www.github.com/nixtodo/oshwdrv
*               for description of hardware and firmware.
*
* Usage:  halcmd loadrt hal_oshwdrv port_addr=<addr1>[,addr2[,addr3]] [epp_dir=<1 | 0>]
*               where 'addr1', 'addr2', and 'addr3' are the addresses
*               of up to three enhanced parallel ports.
*               If epp_dir=1 is given, then this driver explicitly forces the
*               port direction every time it needs to be changed.
*               This is known to cause at least one PCIe port card to malfunction.
*
* Author: Dirk Radloff based on Pico Systems Inc. (John Kasunich, Jon Elson, Stephen Wille Padnos)
* License: GPL Version 2
*    
* Copyright (c) 2020 All rights reserved.
*
********************************************************************/

/** The driver searches the enhanced parallel port (EPP) at 'port_addr',
    looking for an board. It then exports HAL pins for whatever it
    finds, as well as a pair of functions, one that reads all 
    inputs, and one that writes all outputs.
*/

/** This program is free software; you can redistribute it and/or
    modify it under the terms of version 2 of the GNU General
    Public License as published by the Free Software Foundation.
    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

    THE AUTHORS OF THIS LIBRARY ACCEPT ABSOLUTELY NO LIABILITY FOR
    ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE
    TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of
    harming persons must have provisions for completely removing power
    from all motors, etc, before persons enter any danger area.  All
    machinery must be designed to comply with local and national safety
    codes, and the authors of this software can not, and do not, take
    any responsibility for such compliance.

    This code was written as part of the LinuxCNC HAL project.  For more
    information, go to www.linuxcnc.org.
*/

#include <rtapi_slab.h>     // kmalloc()
#include <rtapi_io.h>       // kmalloc()
#include "rtapi.h"          // RTAPI realtime OS API
#include "rtapi_app.h"      // RTAPI realtime module decls
#include "hal.h"            // HAL public API decls
#include "hal_parport.h"	// HAL parport API decls

// Driver information
MODULE_AUTHOR("Dirk Radloff");
MODULE_DESCRIPTION("HAL driver for OsHwDrv board");
MODULE_LICENSE("GPL");

/***********************************************************************
*                DEFINES (MOSTLY REGISTER ADDRESSES)                   *
************************************************************************/
#define MAX_BUS 	 	3			// Max number of parports (EPP busses)
#define SLOT_SIZE    	32			// Max EPP addresses used
#define EPSILON      	1e-20		// Limit value
#define MODULE_CLOCK	10000000.0	// 10 MHz
#define SECONDARY_CLOCK 2500000.0	// 2.5 MHz

// EPP parport addresses
#define SPPDATA(addr)     addr
#define STATUSPORT(addr)  addr+1
#define CONTROLPORT(addr) addr+2
#define ADDRPORT(addr)    addr+3
#define DATAPORT(addr)    addr+4

// Module IDs
#define MODULE_ID_END		0x00
#define MODULE_ID_DIN		0x01
#define MODULE_ID_DOUT		0x02
#define MODULE_ID_STEPENC	0x03
#define MODULE_ID_AOUT		0x04
#define MODULE_ID_AIN		0x05
#define MODULE_ID_WATCHDOG	0x06
#define MODULE_ID_MSPI		0x07
#define MODULE_ID_PWMO		0x08
#define MODULE_ID_JOGENC	0x09

// Module ID addresses
#define MODULE_ID_WRITE		0x00
#define MODULE_ID_READ		0x00

// Module ID Bits
#define MODULE_ID_ADDR		0x1F
#define MODULE_ID_AREA		0x80
#define MODULE_ID_BITS		0x1F

// Module address area switching
#define MODULE_NORMAL_AREA	(SelWrt (0, MODULE_ID_WRITE, port_addr[bus->busnum]))
#define MODULE_STATIC_AREA	(SelWrt (MODULE_ID_AREA, MODULE_ID_WRITE, port_addr[bus->busnum]))

// Module address length (read & write) 
#define MODULE_RD_DIN		0x01
#define MODULE_WR_DIN		0x00
#define MODULE_RD_DOUT		0x00
#define MODULE_WR_DOUT		0x01
#define MODULE_RD_STEPENC	0x04
#define MODULE_WR_STEPENC	0x04
#define MODULE_RD_AOUT		0x00
#define MODULE_WR_AOUT		0x03
#define MODULE_RD_AIN		0x03
#define MODULE_WR_AIN		0x00
#define MODULE_RD_WDOG		0x01
#define MODULE_WR_WDOG		0x01
#define MODULE_RD_PWMO		0x00
#define MODULE_WR_PWMO		0x02
#define MODULE_RD_JOG		0x02
#define MODULE_WR_JOG		0x01

// Din address offsets
#define DIN_DATA			0x00

// Dout address offsets
#define DOUT_DATA			0x00

// Stepencoder address offsets
#define STEPENC_CONTROL		0x00
#define STEPENC_GEN_LOW		0x01
#define STEPENC_GEN_MID		0x02
#define STEPENC_GEN_HI		0x03
#define STEPENC_WIDTH		0x00
#define STEPENC_SETUP		0x01
#define STEPENC_ENC_LOW		0x00
#define STEPENC_ENC_MID1	0x01
#define STEPENC_ENC_MID2	0x02
#define STEPENC_ENC_HIGH	0x03
#define STEPENC_ENC_INDEX	0x04

// Stepencoder Bits
#define STEPENC_ENC_COI		0x80
#define STEPENC_ENC_LATCH	0x40
#define STEPENC_ENC_CLR		0x20
#define STEPENC_STEP_DIR	0x02
#define STEPENC_STEP_EN		0x01
#define STEPENC_ENC_INX_7	0x80
#define STEPENC_ENC_INX_6	0x40
#define STEPENC_ENC_INX_5	0x20
#define STEPENC_ENC_INX_4	0x10
#define STEPENC_ENC_INX_3	0x08
#define STEPENC_ENC_INX_2	0x04
#define STEPENC_ENC_INX_1	0x02
#define STEPENC_ENC_INX_0	0x01

// Stepencoder hardware limits
#define STEPENC_CLOCK		MODULE_CLOCK
#define STEPENC_WIDTH_MIN	100
#define STEPENC_WIDTH_MAX	25500
#define STEPENC_SETUP_MIN	100
#define STEPENC_SETUP_MAX	25500
#define STEPENC_CNT_BITS	24

// Aout address offsets
#define AOUT_DATA_0			0x00
#define AOUT_DATA_1			0x01
#define AOUT_HIGH			0x02

// Aout Bits
#define AOUT_HBITS_0		0x03
#define AOUT_HBITS_1		0x0C
#define AOUT_ENABLE			0x80

// Aout hardware limits
#define AOUT_SCALE_RATIO    100.0
#define AOUT_MIN_OUTPUT		0
#define AOUT_MAX_OUTPUT		1000

// Ain address offsets
#define AIN_DATA_0			0x00
#define AIN_DATA_1			0x01
#define AIN_HIGH			0x02

// Ain Bits
#define AIN_HBITS_0			0x0F
#define AIN_DATA_0_SHIFT	0
#define AIN_HBITS_1			0xF0
#define AIN_DATA_1_SHIFT	4

// Watchdog address offsets
#define WATCHDOG_CONFIG		0x00
#define WATCHDOG_STATUS		0x00

// Watchdog Bits
#define WATCHDOG_CLEAR		0x01
#define WATCHDOG_ID			0x7E
#define WATCHDOG_RESET		0x80
#define WATCHDOG_TIMEOUT	0x01
#define WATCHDOG_ACTIVE		0x80

// Watchdog values
#define WATCHDOG_ID_VALUE	0x56

// PWMo address offsets
#define PWMO_PULSE_LOW		0x00
#define PWMO_PULSE_HIGH		0x01
#define PWMO_PERIOD_LOW		0x00
#define PWMO_PERIOD_HIGH	0x01

// PWMo hardware limits
#define PWMO_CLOCK			SECONDARY_CLOCK
#define PWMO_MIN_FREQ		(PWMO_CLOCK / 65535)
#define PWMO_MAX_FREQ		(PWMO_CLOCK / 100)
#define PWMO_MIN_PERIOD		0.2
#define PWMO_MAX_PERIOD		99.9
#define PWMO_SCALE_RATIO    100.0

// Jog-Encoder address offsets
#define JOG_COUNT_LOW		0x00
#define JOG_COUNT_HIGH		0x01
#define JOG_CONTROL			0x00

// Jog-Encoder Bits
#define JOG_CLEAR			0x20
#define JOG_LATCH			0x40

// Template address offsets
// Template Bits
// Template hardware limits

/***********************************************************************
*                       STRUCTURE DEFINITIONS                          *
************************************************************************/
// This structure contains the runtime data for a digital input
typedef struct din_s {
    unsigned char rd_addr;	 // Base address for reading data
    hal_bit_t *data[8];      // HAL input pin value
    hal_bit_t *data_not[8];  // HAL inverted input pin value
} din_t;

// This structure contains the runtime data for a digital output
typedef struct dout_s {
    unsigned char wr_addr;	// Base address for writing data
    hal_bit_t *data[8];		// HAL output pin value
    hal_bit_t invert[8];    // HAL parameter to invert output pin
} dout_t;

// This structure contains the runtime data for a single stepencoder
typedef struct stepenc_s {
    unsigned char rd_addr;			// Base address for reading data
    unsigned char wr_addr;			// Base address for writing data
	unsigned char master;			// Number of the corresponding Stepencoder
	unsigned char master_index;		// Index Bit position of the corresponding Stepencoder
	hal_bit_t     *step_enable;		// Stepgen enable pin
    hal_float_t   *step_vel;		// Stepgen velocity command pin
    hal_float_t   step_scale;		// Stepgen scaling parameter (vel to Hz)
    hal_float_t   step_max_vel;		// Stepgen max velocity limit
    hal_float_t   step_freq;		// Stepgen frequency (velocity cmd scaled to Hz)
    hal_u32_t     step_pulse_width;	// Stepgen pulse width (in nanoseconds)
	hal_u32_t     last_pulse_width;	// Last written step pulse width value
    hal_u32_t     step_setup_time;	// Stepgen setup time (time between step pulses and dir changes, in nanoseconds)
	hal_u32_t     last_setup_time;	// Last setup time value written
    hal_s32_t     *count;			// Encoder raw (unscaled) encoder counts
    hal_s32_t     *delta;			// Encoder raw (unscaled) delta counts since last read
    hal_float_t   enc_scale;		// Encoder scaling parameter (counts to position)
    hal_float_t   *position;		// Encoder scaled position
    hal_float_t   *enc_vel;			// Encoder scaled velocity
    hal_bit_t     *index;			// Encoder index pules flag
    hal_bit_t     *index_enable;	// Encoder enable index pulse to reset encoder count
} stepenc_t;

// This structure contains the runtime data for two analog outputs (PWM) 
typedef struct aout_s {
    unsigned char wr_addr;	// Base address for writing data
	hal_bit_t     *enable;	// Enable pin
    hal_float_t   *value0;	// Output 0 value
    hal_float_t   offset0;	// Offset 0, will be added to value before scaling
    hal_float_t   scale0;	// Scaling 0 parameter (Value to 0 - 10 Volt)
    hal_float_t   *value1;	// Output 1 value
    hal_float_t   offset1;	// Offset 1, will be added to value before scaling
    hal_float_t   scale1;	// Scaling 1 parameter (Value to 0 - 10 Volt)
} aout_t;

// This structure contains the runtime data for two analog inputs (12 Bit ADC) 
typedef struct ain_s {
    unsigned char rd_addr;	// Base address for reading data
	hal_float_t   *value0;	// Input 0 value
    hal_float_t   offset0;	// Offset 0, will be added to value before scaling
    hal_float_t   scale0;	// Scaling 0 parameter
    hal_float_t   *value1;	// Input 1 value
    hal_float_t   offset1;	// Offset 1, will be added to value before scaling
    hal_float_t   scale1;	// Scaling 1 parameter
} ain_t;

// This structure contains the runtime data for a PWM output 
typedef struct pwmo_s {
    unsigned char wr_addr;			// Base address for writing data
	hal_bit_t     *enable;			// Enable pin
    hal_float_t   *value;			// Output value
    hal_float_t   offset;			// Offset, will be added to value before scaling
    hal_float_t   scale;			// Scaling parameter (Value to 0 - 100% PWM duty cycle)
    hal_float_t   frequency;		// Frequency of the PWM output
	hal_float_t   last_frequency;	// Last PWM frequency written
	unsigned int  divisor;			// Current divisor (Clock / frequency)
} pwmo_t;

// This structure contains the runtime data for a Jog-Encoder 
typedef struct jog_s {
    unsigned char rd_addr;	// Base address for reading data
	unsigned char wr_addr;	// Base address for writing data
	hal_bit_t     *clear;	// Clear the encoder counter
    hal_s32_t     *count;	// Encoder raw (unscaled) encoder counts
    hal_s32_t     *delta;	// Encoder raw (unscaled) delta counts since last read
} jog_t;

// This structure contains the runtime data for the watchdog
typedef struct watchdog_s {
    unsigned char rd_addr;	// Base address for reading data
	unsigned char wr_addr;	// Base address for writing data
	hal_bit_t     *timeout;	// A timeout indicator
	hal_bit_t     *estop;	// Emergency Stop input
} watchdog_t;

// This structure contains the runtime data for one complete EPP bus
typedef struct bus_data_s {
	int busnum;							// Index of parport[] this struct belongs to
	unsigned char read_end_addr;		// Last EPP address needed to be readed from
    unsigned char rd_buf[SLOT_SIZE];	// Cached data read from EPP bus
    unsigned char write_end_addr;		// Last EPP address needed to be written to
    unsigned char wr_buf[SLOT_SIZE];	// cached data to be written to EPP bus
    din_t         *din;					// Ptr to shared memory data for digital inputs
    unsigned char num_din;				// Number of digital input modules
    dout_t        *dout;				// Ptr to shared memory data for digital outputs
    unsigned char num_dout;				// Number of digital output modules
    stepenc_t     *stepenc;				// Ptr to shared memory data for stepencoders
    unsigned char num_stepenc;			// Number of stepencoder modules
    aout_t        *aout;				// Ptr to shared memory data for aout
    unsigned char num_aout;				// Number of aout modules
    ain_t         *ain;					// Ptr to shared memory data for ain
    unsigned char num_ain;				// Number of ain modules
    pwmo_t        *pwmo;				// Ptr to shared memory data for pwmo
    unsigned char num_pwmo;				// Number of pwmo modules
	jog_t         *jog;					// Ptr to shared memory data for Jog-Encoders
    unsigned char num_jog;				// Number of jog modules
	watchdog_t    *watchdog;			// Ptr to shared memory data for Watchdog
} bus_data_t;

/***********************************************************************
*                          GLOBAL VARIABLES                            *
************************************************************************/
static bus_data_t *bus_array[MAX_BUS];	// Array of all parports data
static int comp_id;      				// Component ID returned from HAL
static long read_period; 				// Makes real time period available to called functions

hal_parport_t port_registration[MAX_BUS];

int port_addr[MAX_BUS] = {0x378, [1 ... MAX_BUS-1] = -1}; // default, 1 bus at 0x0378
RTAPI_MP_ARRAY_INT(port_addr, MAX_BUS, "port address(es) for EPP bus(es)");

int epp_dir[MAX_BUS] = {0, [1 ... MAX_BUS-1] = 0};
RTAPI_MP_ARRAY_INT(epp_dir, MAX_BUS, "EPP is commanded port direction");

/***********************************************************************
*                    REALTIME FUNCTION DECLARATIONS                    *
************************************************************************/
static void read_all (void *arg, long period);
static void write_all (void *arg, long period);
static void read_din (bus_data_t *bus);
static void write_dout (bus_data_t *bus);
static void read_stepenc (bus_data_t *bus);
static unsigned int ns2cp (hal_u32_t *pns, unsigned int min_ns, unsigned int max_ns);
static void write_stepenc (bus_data_t *bus);
static void write_aout (bus_data_t *bus);
static void read_ain (bus_data_t *bus);
static void write_pwmo (bus_data_t *bus);
static void read_jog (bus_data_t *bus);
static void write_jog (bus_data_t *bus);
static void read_watchdog (bus_data_t *bus);
static void write_watchdog (bus_data_t *bus);

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/
static int module_base_addr (bus_data_t *bus, int moduleid, int number);
static int count_modules_id (bus_data_t *bus, int moduleid);
static int export_din (bus_data_t *bus);
static int export_dout (bus_data_t *bus);
static int export_stepencoder (bus_data_t *bus);
static int export_aout (bus_data_t *bus);
static int export_ain (bus_data_t *bus);
static int export_pwmo (bus_data_t *bus);
static int export_jog (bus_data_t *bus);
static int export_watchdog (bus_data_t *bus);

/***********************************************************************
*                  REALTIME I/O FUNCTION DECLARATIONS                  *
************************************************************************/
static int ClrTimeout (unsigned int port_addr);
static unsigned short SelRead (unsigned char epp_addr, unsigned int port_addr, int eppdir);
static unsigned short ReadMore (unsigned int port_addr);
static void SelWrt (unsigned char byte, unsigned char epp_addr, unsigned int port_addr);
static void WrtMore (unsigned char byte, unsigned int port_addr);

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/
static int read_module_ids(bus_data_t *bus);
void rtapi_app_exit(void);

int rtapi_app_main (void)
{
    int msg, rv, busnum;
    bus_data_t *bus;
    char buf[HAL_NAME_LEN + 1];

    /* connect to the HAL */
    comp_id = hal_init("hal_oshwdrv");
    
    if (comp_id < 0){
        rtapi_print_msg(RTAPI_MSG_ERR, "oshwdrv: ERROR: hal_init() failed\n");
        return -1;
    }
    
    rtapi_print_msg(RTAPI_MSG_INFO, "oshwdrv: installing driver\n");
    
    // Set message level to RTAPI_MSG_INFO, save old for restore
    msg = rtapi_get_msg_level();
    rtapi_set_msg_level(RTAPI_MSG_INFO);
    
    // Check all parports are available
    for (busnum = 0; busnum < MAX_BUS; busnum++){
        rtapi_print_msg(RTAPI_MSG_INFO, "oshwdrv: Parport %d epp_dir = %d\n", busnum, epp_dir[busnum]);
        bus_array[busnum] = NULL;
        
        /* check to see if a port address was specified */
        if (port_addr[busnum] == -1){
            continue;
        }
        
        rv = hal_parport_get(comp_id, &port_registration[busnum], port_addr[busnum], 0, PARPORT_MODE_EPP);

        if(rv < 0){
			rtapi_print_msg(RTAPI_MSG_ERR, "oshwdrv: ERROR: hal_parport_get() failed at %d\n", busnum);
            rtapi_app_exit();
            return rv;
        }

        port_addr[busnum] = port_registration[busnum].base;
        
        if(port_registration[busnum].base_hi)
            rtapi_outb(0x80, port_registration[busnum].base_hi + 2);

        /* allocate memory for bus data - this is not shared memory */
        bus = rtapi_kmalloc(sizeof(bus_data_t), RTAPI_GFP_KERNEL);
        
        if (bus == 0){
            rtapi_print_msg(RTAPI_MSG_ERR, "oshwdrv: ERROR: kmalloc() failed\n");
            rtapi_app_exit();
            return -1;
        }
        
        bus_array[busnum] = bus;

        /* clear this bus data structure */
        bus->busnum = busnum;
        bus->read_end_addr = 1;
        bus->write_end_addr = 1;
		bus->num_din = 0;
		bus->din = NULL;
		bus->num_dout = 0;
		bus->dout = NULL;
		bus->num_stepenc = 0;
		bus->stepenc = NULL;
		bus->num_aout = 0;
		bus->aout = NULL;
		bus->num_pwmo = 0;
		bus->pwmo = NULL;
		bus->num_jog = 0;
		bus->jog = NULL;
		bus->watchdog = NULL;
		
		// Read the Module IDs at this bus
		rv = read_module_ids(bus);
		
		if (rv != 0){
            rtapi_app_exit();
            return -1;
		}
		
		// Do all module specific init and configuration
		rv = 0;
		rv += export_din(bus);
		rv += export_dout(bus);
		rv += export_stepencoder(bus);
		rv += export_aout(bus);
		rv += export_ain(bus);
		rv += export_pwmo(bus);
		rv += export_jog(bus);
		rv += export_watchdog(bus);
		
		rtapi_print_msg(RTAPI_MSG_INFO, "oshwdrv: Last read address %d\n", bus->read_end_addr);
		rtapi_print_msg(RTAPI_MSG_INFO, "oshwdrv: Last write address %d\n", bus->write_end_addr);
		
		// Check for failures at the module export
		if (rv != 0){
            rtapi_print_msg(RTAPI_MSG_ERR, "oshwdrv: ERROR: Failure at module init\n");
            rtapi_app_exit();
			return -1;
        }
		
        // Export read and write functions for this bus to the HAL
        rtapi_snprintf(buf, sizeof(buf), "oshwdrv.%d.read", busnum);
        rv = hal_export_funct(buf, read_all, &(bus_array[busnum]), 1, 0, comp_id);
        
        if (rv != 0){
            rtapi_print_msg(RTAPI_MSG_ERR, "oshwdrv: ERROR: read funct export failed\n");
            rtapi_app_exit();
			return -1;
        }
        
        rtapi_snprintf(buf, sizeof(buf), "oshwdrv.%d.write", busnum);
        rv = hal_export_funct(buf, write_all, &(bus_array[busnum]), 1, 0, comp_id);
        
        if (rv != 0){
            rtapi_print_msg(RTAPI_MSG_ERR, "oshwdrv: ERROR: write funct export failed\n");
            rtapi_app_exit();
			return -1;
        }
		
		rtapi_print_msg(RTAPI_MSG_INFO, "oshwdrv: Parport %d completed\n", busnum);
    }
    
    rtapi_print_msg(RTAPI_MSG_INFO, "oshwdrv: Driver installed\n");
    rtapi_set_msg_level(msg);
    hal_ready(comp_id);
    return 0;
}

static int read_module_ids (bus_data_t *bus)
{
	int cnt, id;
	int busnum = bus->busnum;
	
	// Read all module IDs starting with module 0
	for (cnt = 0; cnt < SLOT_SIZE; cnt++){
		SelWrt(cnt, 0x00, port_addr[busnum]); // Write next ID address value to EPP address 0x00
		id = SelRead(0x00, port_addr[busnum], epp_dir[busnum]); // Read the ID value from EPP address 0x00
		rtapi_print_msg(RTAPI_MSG_INFO, "oshwdrv: Module ID %d\n", id);
		
		// Value 0xFF will not be used so we can detect them. Also returning the written value indicates missing hardware
		if (id == 0xFF){
			rtapi_print_msg(RTAPI_MSG_ERR, "oshwdrv: No Module ID found %d\n", id);
			return -1;
		}
		
		bus->rd_buf[cnt] = id;
		
		// If we read a value without Bit 7 set, we found the end of the Module IDs
		if ((id & MODULE_ID_BITS) == MODULE_ID_END){
			rtapi_print_msg(RTAPI_MSG_INFO, "oshwdrv: Last Module ID found\n");
			return 0;
		}
	}
	
	return 0;
}

void rtapi_app_exit (void)
{
    int busnum, n;
    bus_data_t *bus;
    
    for (busnum = 0; busnum < MAX_BUS; busnum++){        
        // Check to see if memory was allocated for this bus
        if (bus_array[busnum] != NULL ) {
            bus = bus_array[busnum];
            
            // Mark it invalid so RT code won't access
            bus_array[busnum] = NULL;
            
            // Write all zeros to the hardware to switch off all stuff
            SelWrt(0, 0x01, port_addr[busnum]);
            
            for (n = 2; n < SLOT_SIZE; n++){
				WrtMore(0, port_addr[busnum]);
			}
            
			// Free this bus_data_t memory block
			rtapi_kfree(bus);
        }
        
        // If a parport were requested, release it
        hal_parport_release(&port_registration[busnum]);
    }

    // Disconnect from HAL
    hal_exit(comp_id);
}

/***********************************************************************
*                         REALTIME FUNCTIONS                           *
************************************************************************/
static void read_all (void *arg, long period)
{
    bus_data_t *bus;
    int n;
	
	bus = *(bus_data_t **)(arg); // Get pointer to bus data structure
    read_period = period; // Make thread period available to called functions
        
    // Test to make sure it hasn't been freed
    if (bus == NULL){
        return;
    }
    
	// Send a latch strobe to the stepencoders and or jog encoders
	// Will be done by FPGA firmware. (Start reading at address 1 will trigger it.)
	// So no module with an encoder function should be placed at address 1!
	
	// Fetch all data from EPP to cache
	bus->rd_buf[1] = SelRead(0x01, port_addr[bus->busnum], epp_dir[bus->busnum]);
	
	for (n = 2; n <= bus->read_end_addr; n++){
		bus->rd_buf[n] = ReadMore(port_addr[bus->busnum]);
	}
	
	// Call all functions for read (input data)
	read_din(bus);
	read_stepenc(bus);
	read_ain(bus);
	read_jog(bus);
	read_watchdog(bus);
}

static void write_all(void *arg, long period)
{
    bus_data_t *bus;
    int n;
	
	bus = *(bus_data_t **)(arg); // Get pointer to bus data structure
    read_period = period; // Make thread period available to called functions
        
    // Test to make sure it hasn't been freed
    if (bus == NULL){
        return;
    }
	
	// Call all functions for write (output data)
	write_dout(bus);
	write_stepenc(bus);
	write_aout(bus);
	write_pwmo(bus);
	write_jog(bus);
	write_watchdog(bus);
	
	// Write data from cache to EPP
	SelWrt(bus->wr_buf[1], 0x01, port_addr[bus->busnum]);
	
	for (n = 2; n <= bus->write_end_addr; n++){
		WrtMore(bus->wr_buf[n], port_addr[bus->busnum]);
	}
}

static void read_din (bus_data_t *bus)
{
	int n, b;
	din_t *di;
	unsigned char indata, mask;
	
	// Test to make sure it hasn't been freed
	if (bus->din == NULL){
		return;
	}
	
	// Loop through all Din modules
	for (n = 0; n < bus->num_din; n++){
		di = &(bus->din[n]);
		
		// Read the 8 inputs Bits
		indata = bus->rd_buf[(di->rd_addr + DIN_DATA)];
		mask = 0x01;
		
		for (b = 0; b < 8; b++){
			*(di->data[b]) = indata & mask;
			*(di->data_not[b]) = !(indata & mask);
			mask <<= 1;
		}
	}
}

static void write_dout (bus_data_t *bus)
{
	int n, b;
	dout_t *dou;
    unsigned char outdata, mask;
	
	// Test to make sure it hasn't been freed
	if (bus->dout == NULL){
		return;
	}
	
	// Loop through all Dout modules
	for (n = 0; n < bus->num_dout; n++){
		dou = &(bus->dout[n]);
		
		// Assemble output Byte from 8 source variables
		outdata = 0x00;
		mask = 0x01;
		
		for (b = 0; b < 8; b++){
			// Get the data and the invert Bit
			if (((*(dou->data[b]) != 0) && (dou->invert[b] == 0)) || ((*(dou->data[b]) == 0) && (dou->invert[b] != 0))){
				outdata |= mask;
			}
			
			mask <<= 1;
		}
		
		// Write this Byte to the cache
		bus->wr_buf[(dou->wr_addr + DOUT_DATA)] = outdata;
	}
}

static void read_stepenc (bus_data_t *bus)
{
	int n, addr, mask;
	signed long newpos;
	stepenc_t *se, *se2;
	
	// Test to make sure it hasn't been freed
	if (bus->stepenc == NULL){
		return;
	}
	
	// Loop through all Stepencoder modules
	for (n = 0; n < bus->num_stepenc; n++){
		se = &(bus->stepenc[n]);
		addr = se->rd_addr;
		
		// Get the new position (in counts)
		newpos = (signed long)bus->rd_buf[(addr + STEPENC_ENC_LOW)];
		newpos += ((signed long)(bus->rd_buf[(addr + STEPENC_ENC_MID1)]) << 8);
		newpos += ((signed long)(bus->rd_buf[(addr + STEPENC_ENC_MID2)]) << 16);
		newpos += ((signed long)(bus->rd_buf[(addr + STEPENC_ENC_HIGH)]) << 24);
		
		// Set HAL delta, count pins
		*(se->delta) = newpos - *(se->count);
		*(se->count) = newpos;
		
		// Limit HAL enc_scale pin
		if (se->enc_scale < 0.0){
			if (se->enc_scale > -EPSILON)
				se->enc_scale = -1.0;
		}
		else {
			if (se->enc_scale < EPSILON)
				se->enc_scale = 1.0;
		}
		
		// Set HAL position pin
		*(se->position) = (hal_s32_t)newpos / se->enc_scale;
		
		// Set HAL velocity pin
		*(se->enc_vel) = *(se->delta) / (read_period * 1e-9 * se->enc_scale);
		
		// Set HAL index pin
		se2 = &(bus->stepenc[se->master]);
		addr = se2->rd_addr + STEPENC_ENC_INDEX;
		mask = se->master_index;
		
		*(se->index) = bus->rd_buf[addr] & mask;
		
		// Reset Index-enable HAL pin if an index signal was found
		if ((*(se->index) != 0) && (*(se->index_enable) != 0)){
			*(se->index_enable) = 0;
		}
	}
}

// Fetch a time parameter (in nS), make sure it is a multiple of 100ns,
// and is between min_ns and max_ns, and return the value in 10MHz clock pulses.
static unsigned int ns2cp (hal_u32_t *pns, unsigned int min_ns, unsigned int max_ns)
{
	int ns, cp;
	
	ns = *pns;
	
	// Limit to minimum value
	if (ns < min_ns)
		ns = min_ns;
	
	// Limit to maximum value
	if (ns > max_ns)
		ns = max_ns;
	
	// Round to full 100ns and return this value
	cp = ns / 100;
	ns = cp * 100;
	*pns = ns;
	return cp;
}

static void write_stepenc (bus_data_t *bus)
{
	int n, addr, run, reverse;
	stepenc_t *se;
	double abs_scale, max_freq, freq;
	unsigned int divisor;
	unsigned char control_byte;
	
	// Test to make sure it hasn't been freed
	if (bus->stepenc == NULL){
		return;
	}
	
	// Loop through all Stepencoder modules
	for (n = 0; n < bus->num_stepenc; n++){
		se = &(bus->stepenc[n]);
		addr = se->wr_addr;
		
		// Pulse width or setup time changed?
		if ((se->step_pulse_width != se->last_pulse_width) || (se->step_setup_time != se->last_setup_time)){
			// Switch to static data area
			MODULE_STATIC_AREA;
			
			// Save new pulse width value
			se->last_pulse_width = ns2cp(&(se->step_pulse_width), STEPENC_WIDTH_MIN, STEPENC_WIDTH_MAX);
			SelWrt (se->last_pulse_width, (addr + STEPENC_WIDTH), port_addr[bus->busnum]);
			
			// Save new setup time value
			se->last_setup_time = ns2cp(&(se->step_setup_time), STEPENC_SETUP_MIN, STEPENC_SETUP_MAX);
			SelWrt (se->last_setup_time, (addr + STEPENC_SETUP), port_addr[bus->busnum]);
			
			// Switch back to normal data area
			MODULE_NORMAL_AREA;
		}
		
		// Calculate max frequency
		max_freq = STEPENC_CLOCK / (se->last_pulse_width + (STEPENC_WIDTH_MIN / 100));
		
		// Validate the HAL scale value
		if (se->step_scale < 0.0){
			if (se->step_scale > -EPSILON){
				// Too small, divide by zero is bad
				se->step_scale = -1.0;
			}
			
			abs_scale = -se->step_scale;
		}
		else {
			if (se->step_scale < EPSILON){
				se->step_scale = 1.0;
			}
			
			abs_scale = se->step_scale;
		}
		
		// Check for user specified max velocity
		if (se->step_max_vel <= 0.0){
			// Set to zero if negative, and ignore if zero
			se->step_max_vel = 0.0;
		}
		else {
			// Parameter is non-zero and positive, compare to max_freq
			if ((se->step_max_vel * abs_scale) > max_freq){
				// Parameter is too high, lower it
				se->step_max_vel = max_freq / abs_scale;
			}
			else {
				// Lower max_freq to match parameter
				max_freq = se->step_max_vel * abs_scale;
			}
		}
		
		// Calculate desired frequency
		freq = *(se->step_vel) * se->step_scale;
		
		// Should the stepgen running?
		if (*(se->step_enable) != 0){
			run = 1;
		}
		else {
			run = 0;
		}
		
		// Negative frequency?
		reverse = 0;
		
		if (freq < 0.0){
			freq = -freq;
			reverse = 1;
		}
		
		// Apply limits
		if (freq > max_freq){
			freq = max_freq;
			divisor = STEPENC_CLOCK / freq;
			// Subtract step pulse width 
			divisor -= ns2cp(&(se->step_pulse_width), STEPENC_WIDTH_MIN, STEPENC_WIDTH_MAX);
		}
		else if (freq < (STEPENC_CLOCK / ((1 << STEPENC_CNT_BITS) - 1))){
			// Frequency would result in a divisor greater than 2^24-1
			freq = 0.0;
			divisor = (1 << STEPENC_CNT_BITS) - 1;
			
			// The only way to get zero is to turn it off
			run = 0;
		}
		else {
			// Calculate divisor, round to nearest instead of truncating
			divisor = (STEPENC_CLOCK / freq) + 0.5;
			// Calculate actual frequency (due to divisor roundoff)
			freq = STEPENC_CLOCK / divisor;
			// Subtract step pulse width 
			divisor -= ns2cp(&(se->step_pulse_width), STEPENC_WIDTH_MIN, STEPENC_WIDTH_MAX);
		}
		
		// Set enable and dir Bit in the control register, save the frequency
		control_byte = bus->wr_buf[(addr + STEPENC_CONTROL)];
		
		if (run){
			control_byte |= STEPENC_STEP_EN;
		}
		else {
			control_byte &= ~STEPENC_STEP_EN;
		}
		
		if (reverse) {
			se->step_freq = -freq;
			control_byte &= ~STEPENC_STEP_DIR;
		}
		else {
			se->step_freq = freq;
			control_byte |= STEPENC_STEP_DIR;
		}
		
		// Set clear on index Bit if index-enable is active
		if (*(se->index_enable) != 0){
			control_byte |= STEPENC_ENC_COI;
		}
		else {
			control_byte &= ~STEPENC_ENC_COI;
		}
		
		bus->wr_buf[(addr + STEPENC_CONTROL)] = control_byte;
		
		// Write new counter value
		bus->wr_buf[(addr + STEPENC_GEN_LOW)] = divisor & 0xff;
		divisor >>= 8;
		bus->wr_buf[(addr + STEPENC_GEN_MID)] = divisor & 0xff;
		divisor >>= 8;
		bus->wr_buf[(addr + STEPENC_GEN_HI)] = divisor & 0xff;
	}
}

static void write_aout (bus_data_t *bus)
{
	int n, addr;
	aout_t *ao;
	int val0, val1;
	
	// Test to make sure it hasn't been freed
	if (bus->aout == NULL){
		return;
	}
	
	// Loop through all Aout modules
	for (n = 0; n < bus->num_aout; n++){
		ao = &(bus->aout[n]);
		addr = ao->wr_addr;
		
		// Check enable pin
		if (*(ao->enable) == 0){
			bus->wr_buf[(addr + AOUT_DATA_0)] = 0;
			bus->wr_buf[(addr + AOUT_DATA_1)] = 0;
			bus->wr_buf[(addr + AOUT_HIGH)] = 0;			
			continue;
		}
		
		// Calculate data
		val0 = (int)(((*(ao->value0)) + ao->offset0) / (ao->scale0 / AOUT_SCALE_RATIO));
		val1 = (int)(((*(ao->value1)) + ao->offset1) / (ao->scale1 / AOUT_SCALE_RATIO));
		
		// Check for negative slope
		if (val0 < 0){
			val0 += 1000;
		}

		if (val1 < 0){
			val1 += 1000;
		}
		
		// Limit calculated data
		if (val0 < AOUT_MIN_OUTPUT){
			val0 = AOUT_MIN_OUTPUT;
		}
		else if (val0 > AOUT_MAX_OUTPUT){
			val0 = AOUT_MAX_OUTPUT;
		}
		
		if (val1 < AOUT_MIN_OUTPUT){
			val1 = AOUT_MIN_OUTPUT;
		}
		else if (val1 > AOUT_MAX_OUTPUT){
			val1 = AOUT_MAX_OUTPUT;
		}
		
		// Output data
		bus->wr_buf[(addr + AOUT_DATA_0)] = val0 & 0xFF;
		bus->wr_buf[(addr + AOUT_DATA_1)] = val1 & 0xFF;
		bus->wr_buf[(addr + AOUT_HIGH)] = AOUT_ENABLE | ((val1 >> 6) & 0x0C) | ((val0 >> 8) & 0x03);
	}
}

static void read_ain (bus_data_t *bus)
{
	int n, addr;
	ain_t *ai;
	unsigned int val0, val1;
	
	// Test to make sure it hasn't been freed
	if (bus->ain == NULL){
		return;
	}
	
	// Loop through all Aout modules
	for (n = 0; n < bus->num_ain; n++){
		ai = &(bus->ain[n]);
		addr = ai->rd_addr;
		
		// Read data
		val0 = (((bus->rd_buf[(addr + AIN_HIGH)] & AIN_HBITS_0) >> AIN_DATA_0_SHIFT) * 256) + bus->rd_buf[(addr + AIN_DATA_0)];
		val1 = (((bus->rd_buf[(addr + AIN_HIGH)] & AIN_HBITS_1) >> AIN_DATA_1_SHIFT) * 256) + bus->rd_buf[(addr + AIN_DATA_1)];
		
		// Calculate data
		*(ai->value0) = ((hal_float_t)val0 * ai->scale0) - ai->offset0;
		*(ai->value1) = ((hal_float_t)val1 * ai->scale1) - ai->offset1;
	}
}

static void write_pwmo (bus_data_t *bus)
{
	int n, addr;
	pwmo_t *pw;
	float val;
	unsigned int divisor, period;
	
	// Test to make sure it hasn't been freed
	if (bus->pwmo == NULL){
		return;
	}
	
	// Loop through all PWMo modules
	for (n = 0; n < bus->num_pwmo; n++){
		pw = &(bus->pwmo[n]);
		addr = pw->wr_addr;
		
		// PWM frequency changed?
		if (pw->frequency != pw->last_frequency){
			// Check limits
			if (pw->frequency < PWMO_MIN_FREQ){
				pw->frequency = PWMO_MIN_FREQ;
			}
			else if (pw->frequency > PWMO_MAX_FREQ){
				pw->frequency = PWMO_MAX_FREQ;
			}
			
			// Calculate new divisor and return the real PWM frequency (due to rounding)
			divisor = (unsigned int)(PWMO_CLOCK / pw->frequency);
			pw->divisor = divisor;
			pw->frequency = (hal_float_t)(PWMO_CLOCK / divisor);
			
			// Switch to static data area
			MODULE_STATIC_AREA;
			
			// Save new frequency value
			pw->last_frequency = pw->frequency;
			SelWrt((divisor & 0xFF), (addr + PWMO_PERIOD_LOW), port_addr[bus->busnum]);
			divisor >>= 8;
			SelWrt((divisor & 0xFF), (addr + PWMO_PERIOD_HIGH), port_addr[bus->busnum]);
			
			// Switch back to normal data area
			MODULE_NORMAL_AREA;
		}
		
		// Enable pin?
		if (*(pw->enable) == 0){
			bus->wr_buf[(addr + PWMO_PULSE_LOW)] = 0;
			bus->wr_buf[(addr + PWMO_PULSE_HIGH)] = 0;
		}
		else {
			// Calculate data
			val = ((*(pw->value)) + pw->offset) / pw->scale;
			
			// Check for negative slope
			if (val < 0.00001){ // Used to prevent -0.0 as negative number
				val += 100.0;
			}
			
			// Check limits
			if (val < PWMO_MIN_PERIOD){
				val = PWMO_MIN_PERIOD;
			}
			else if (val > PWMO_MAX_PERIOD){
				val = PWMO_MAX_PERIOD;
			}
			
			// Calculate new period
			period = (unsigned int)((pw->divisor * (val / PWMO_SCALE_RATIO)) - 1.0);
			bus->wr_buf[(addr + PWMO_PULSE_LOW)] = period & 0xFF;
			period >>= 8;
			bus->wr_buf[(addr + PWMO_PULSE_HIGH)] = period & 0xFF;
		}
	}
}

static void read_jog (bus_data_t *bus)
{
	int n, addr;
	hal_s32_t newpos;
	jog_t *jo;
	
	// Test to make sure it hasn't been freed
	if (bus->jog == NULL){
		return;
	}
	
	// Loop through all Jog-Encoders modules
	for (n = 0; n < bus->num_jog; n++){
		jo = &(bus->jog[n]);
		addr = jo->rd_addr;
		
		// Get the new position (in counts)
		newpos =  (hal_s32_t)(bus->rd_buf[(addr + JOG_COUNT_LOW)]);
		newpos += (((hal_s32_t)(bus->rd_buf[(addr + JOG_COUNT_HIGH)])) << 8);
		
		// Read a negative number?
		if (bus->rd_buf[(addr + JOG_COUNT_HIGH)] & 0x80){
			newpos += 0xFFFF0000;
		}
		
		// Set HAL delta, count pins
		*(jo->delta) = newpos - *(jo->count);
		*(jo->count) = newpos;
	}
}

static void write_jog (bus_data_t *bus)
{
	int n, addr;
	jog_t *jo;
	unsigned char control_byte;
	
	// Test to make sure it hasn't been freed
	if (bus->jog == NULL){
		return;
	}
	
	// Loop through all Jog-Encoders modules
	for (n = 0; n < bus->num_jog; n++){
		jo = &(bus->jog[n]);
		addr = jo->wr_addr;
		
		control_byte = bus->wr_buf[(addr + JOG_CONTROL)];
		
		// Clear the jog encoder counter?
		if (*(jo->clear) == 0){
			control_byte |= JOG_CLEAR; // Clear at 0 to simplify HAL wiring
		}
		else {
			control_byte &= ~JOG_CLEAR;
		}
		
		bus->wr_buf[(addr + JOG_CONTROL)] = control_byte;
	}
}

static void read_watchdog (bus_data_t *bus)
{
	int addr, status;
	watchdog_t *wd;
	
	// Test to make sure it hasn't been freed
	if (bus->watchdog == NULL){
		return;
	}
	
	wd = bus->watchdog;
	addr = wd->rd_addr;
	
	// Get the status Bits
	status = (bus->rd_buf[(addr + WATCHDOG_STATUS)]);
	
	// Timeout occured?
	if (status & WATCHDOG_TIMEOUT){
		*(wd->timeout) = 1;
	}
	else {
		*(wd->timeout) = 0;
	}
}

static void write_watchdog (bus_data_t *bus)
{
	int addr, status;
	watchdog_t *wd;
	
	// Test to make sure it hasn't been freed
	if (bus->watchdog == NULL){
		return;
	}
	
	wd = bus->watchdog;
	addr = wd->wr_addr;
	
	// Get the status Bits
	status = (bus->rd_buf[(wd->rd_addr + WATCHDOG_STATUS)]);
	
	// Watchdog inactive?
	if ((status & WATCHDOG_ACTIVE) == 0){
		// Start it
		bus->wr_buf[(addr + WATCHDOG_CONFIG)] = WATCHDOG_RESET | WATCHDOG_ID_VALUE | WATCHDOG_CLEAR;
		return;
	}
	
	// Has a timeout occured?
	if (*(wd->timeout) != 0){
		// Yes, wait until estop is low again
		bus->wr_buf[(addr + WATCHDOG_CONFIG)] = WATCHDOG_ID_VALUE; // Write low to Reset and Clear Bits
		
		// Estop inactive?
		if (*(wd->estop) == 0){
			// Reset timeout
			bus->wr_buf[(addr + WATCHDOG_CONFIG)] = WATCHDOG_RESET | WATCHDOG_ID_VALUE;
		}
		
		return;
	}
	
	// Estop inactive?
	if (*(wd->estop) == 0){
		// Yes, clear watchdog counter (toggle Clear Bit)
		if ((bus->wr_buf[(addr + WATCHDOG_CONFIG)] & WATCHDOG_CLEAR) == 0){
			bus->wr_buf[(addr + WATCHDOG_CONFIG)] = WATCHDOG_ID_VALUE | WATCHDOG_CLEAR;
		}
		else {
			bus->wr_buf[(addr + WATCHDOG_CONFIG)] = WATCHDOG_ID_VALUE;
		}
	}
}

/***********************************************************************
*                   LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/
// Get the EPP base address of the requested module ID and number. Read / write dependant
static int module_base_addr (bus_data_t *bus, int moduleid, int number)
{
	int n, r_addr, w_addr, id, cnt, cntse;
	
	// EPP read / write addresses are starting at 0x01 (0x00 is Module ID stuff only)
	r_addr = 0x01;
	w_addr = 0x01;
	cnt = 0;
	cntse = 0;
	
	// Loop through all possible locations
	for (n = 0; n < SLOT_SIZE; n++){
		id = bus->rd_buf[n] & MODULE_ID_BITS;
		
		// Check for end marker
		if (id == MODULE_ID_END){
			rtapi_print_msg(RTAPI_MSG_ERR, "oshwdrv: ERROR: Can not find requested Module number\n");
			return -1;
		}
		
		// Found a correct ID?
		if (id == moduleid){
			// Found requested module number
			if (cnt == number){
				// Set the found values
				switch (id){
					case MODULE_ID_END:
						rtapi_print_msg(RTAPI_MSG_ERR, "oshwdrv: ERROR: Found END Module\n");
						return -1;
						break;
						
					case MODULE_ID_DIN:
						bus->din[number].rd_addr = r_addr;
						break;
						
					case MODULE_ID_DOUT:
						bus->dout[number].wr_addr = w_addr;
						break;
						
					case MODULE_ID_STEPENC:
						bus->stepenc[number].rd_addr = r_addr;
						bus->stepenc[number].wr_addr = w_addr;
						break;
						
					case MODULE_ID_AOUT:
						bus->aout[number].wr_addr = w_addr;
						break;
						
					case MODULE_ID_AIN:
						bus->ain[number].rd_addr = r_addr;
						break;
						
					case MODULE_ID_WATCHDOG:
						bus->watchdog->rd_addr = r_addr;
						bus->watchdog->wr_addr = w_addr;
						break;
						
					case MODULE_ID_PWMO:
						bus->pwmo[number].wr_addr = w_addr;
						break;
					
					case MODULE_ID_JOGENC:
						bus->jog[number].rd_addr = r_addr;
						bus->jog[number].wr_addr = w_addr;
						break;
						
					default:
						rtapi_print_msg(RTAPI_MSG_ERR, "oshwdrv: ERROR: Found unknown module ID\n");
						return -1;
						break;
				}
				
				//rtapi_print_msg(RTAPI_MSG_INFO, "oshwdrv: READ: %d WRITE: %d\n", r_addr, w_addr);
				return 0;
			}
			
			cnt++;
		}
		
		// Count the addresses
		switch (id){
			case MODULE_ID_END:
				rtapi_print_msg(RTAPI_MSG_ERR, "oshwdrv: ERROR: Found END Module\n");
				return -1;
				break;
				
			case MODULE_ID_DIN:
				r_addr += MODULE_RD_DIN;
				w_addr += MODULE_WR_DIN;
				break;
				
			case MODULE_ID_DOUT:
				r_addr += MODULE_RD_DOUT;
				w_addr += MODULE_WR_DOUT;
				break;
				
			case MODULE_ID_STEPENC:
				r_addr += MODULE_RD_STEPENC;
				w_addr += MODULE_WR_STEPENC;
				
				if ((cntse % 8) == 0){ // The first and every 8th Stepencoder has an Encoder Index register
					r_addr++;
				}
				
				cntse++;
				break;
				
			case MODULE_ID_AOUT:
				r_addr += MODULE_RD_AOUT;
				w_addr += MODULE_WR_AOUT;
				break;
				
			case MODULE_ID_AIN:
				r_addr += MODULE_RD_AIN;
				w_addr += MODULE_WR_AIN;
				break;
				
			case MODULE_ID_WATCHDOG:
				r_addr += MODULE_RD_WDOG;
				w_addr += MODULE_WR_WDOG;
				break;
				
			case MODULE_ID_PWMO:
				r_addr += MODULE_RD_PWMO;
				w_addr += MODULE_WR_PWMO;
				break;
			
			case MODULE_ID_JOGENC:
				r_addr += MODULE_RD_JOG;
				w_addr += MODULE_WR_JOG;
				break;
			
			default:
				rtapi_print_msg(RTAPI_MSG_ERR, "oshwdrv: ERROR: Found unknown module ID: %d\n", id);
				return -1;
				break;
		}
	}
	
	return -1;
}

// Get the number of modules of the requested module ID
static int count_modules_id (bus_data_t *bus, int moduleid)
{
	int n, cnt, id;
	
	cnt = 0;
	
	// Loop through all possible locations
	for (n = 0; n < SLOT_SIZE; n++){
		id = bus->rd_buf[n] & MODULE_ID_BITS;
		
		// Found a correct ID?
		if (id == moduleid){
			cnt++;
		}
		
		// Check for end marker
		if (id == MODULE_ID_END){
			return cnt;
		}
	}
	
	return cnt;
}

// Export all digital inputs to the HAL
static int export_din (bus_data_t *bus)
{
    int cnt, retval, n, id, pinnr;
	din_t *di;

	cnt = count_modules_id(bus, MODULE_ID_DIN);
	bus->num_din = cnt;
	rtapi_print_msg(RTAPI_MSG_INFO, "oshwdrv: Exporting digital inputs %d\n", cnt);
	
	// Return if no module was found
	if (cnt < 1){
		return 0;
	}
	
    // Alocate shared memory
    bus->din = hal_malloc(cnt * sizeof(din_t));
    
    if (bus->din == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "oshwdrv: ERROR: hal_malloc() failed\n");
        return -1;
    }
    
	pinnr = 0;
	
	for (id = 0; id < cnt; id++){
		// Loop through all found modules
		di = &(bus->din[id]);
		retval = module_base_addr(bus, MODULE_ID_DIN, id);
		
		// Check for failure
		if (retval != 0){
			return retval;
		}
		
		for (n = 0; n < 8; n++){
			// Export pins for this input
			retval = hal_pin_bit_newf(HAL_OUT, &(di->data[n]), comp_id, "oshwdrv.%d.din.%02d.in", bus->busnum, pinnr);
			
			if (retval != 0){
				return retval;
			}
			
			retval = hal_pin_bit_newf(HAL_OUT, &(di->data_not[n]), comp_id, "oshwdrv.%d.din.%02d.in-not", bus->busnum, pinnr);
			
			if (retval != 0){
				return retval;
			}
			
			// Count Din pins
			pinnr++;
		}
	}
	
	// Extend the EPP read addresses, if needed
	if (bus->read_end_addr < di->rd_addr){
		bus->read_end_addr = di->rd_addr;
	}
	
    return 0;
}

// Export all digital outputs to the HAL
static int export_dout (bus_data_t *bus)
{
    int cnt, retval, n, id, pinnr;
	dout_t *dou;

	cnt = count_modules_id(bus, MODULE_ID_DOUT);
    bus->num_dout = cnt;
    rtapi_print_msg(RTAPI_MSG_INFO, "oshwdrv: Exporting digital outputs %d\n", cnt);
	
	// Return if no module was found
	if (cnt < 1){
		return 0;
	}
	
    // Allocate shared memory
    bus->dout = hal_malloc(cnt * sizeof(dout_t));
    
    if (bus->dout == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "oshwdrv: ERROR: hal_malloc() failed\n");
        return -1;
    }
    
	pinnr = 0;
	
	for (id = 0; id < cnt; id++){
		// Loop through all modules found
		dou = &(bus->dout[id]);
		retval = module_base_addr(bus, MODULE_ID_DOUT, id);
		
		// Check for failure
		if (retval != 0){
			return retval;
		}
		
		SelWrt(0, dou->wr_addr, port_addr[bus->busnum]); // Set all outputs to low
		
		for (n = 0; n < 8; n++){
			// Export pin for this output
			retval = hal_pin_bit_newf(HAL_IN, &(dou->data[n]), comp_id, "oshwdrv.%d.dout.%02d.out", bus->busnum, pinnr);
			
			if (retval != 0) {
				return retval;
			}
			
			// Export parameter for inversion
			retval = hal_param_bit_newf(HAL_RW, &(dou->invert[n]), comp_id, "oshwdrv.%d.dout.%02d-invert", bus->busnum, pinnr);
			
			if (retval != 0) {
				return retval;
			}
			
			dou->invert[n] = 0;
			
			// Count Dout pins
			pinnr++;
		}
	}
	
	// Extend the EPP write addresses, if needed
	if (bus->write_end_addr < dou->wr_addr){
		bus->write_end_addr = dou->wr_addr;
	}
	
    return 0;
}

// Export all Stepencoders to the HAL
static int export_stepencoder (bus_data_t *bus)
{
    int cnt, retval, id;
	stepenc_t *se;

	cnt = count_modules_id(bus, MODULE_ID_STEPENC);
	bus->num_stepenc = cnt;
	rtapi_print_msg(RTAPI_MSG_INFO, "oshwdrv: Exporting Stepencoders %d\n", cnt);
	
	// Return if no module was found
	if (cnt < 1){
		return 0;
	}
	
	// Allocate shared memory
	bus->stepenc = hal_malloc(cnt * sizeof(stepenc_t));
    
	if (bus->stepenc == 0) {
		rtapi_print_msg(RTAPI_MSG_ERR, "oshwdrv: ERROR: hal_malloc() failed\n");
		return -1;
	}
    
	for (id = 0; id < cnt; id++){
		// Loop through all modules found
		se = &(bus->stepenc[id]);
		retval = module_base_addr(bus, MODULE_ID_STEPENC, id);
		
		if (retval != 0){
			return retval;
		}
		
		// Export Stepgen HAL pins
		// Stepgen enable pin
		retval = hal_pin_bit_newf(HAL_IN, &(se->step_enable), comp_id, "oshwdrv.%d.stepenc.%02d.step-enable", bus->busnum, id);
		
		if (retval != 0){
			return retval;
		}
		
		// Stepgen velocity command pin
		retval = hal_pin_float_newf(HAL_IN, &(se->step_vel), comp_id, "oshwdrv.%d.stepenc.%02d.step-velocity", bus->busnum, id);
		
		if (retval != 0){
			return retval;
		}
		
		// Stepgen velocity scaling parameter
		retval = hal_param_float_newf(HAL_RW, &(se->step_scale), comp_id, "oshwdrv.%d.stepenc.%02d.step-scale", bus->busnum, id);
		
		if (retval != 0){
			return retval;
		}
		
		se->step_scale = 1.0;
		
		// Stepgen maximum velocity parameter
		retval = hal_param_float_newf(HAL_RW, &(se->step_max_vel), comp_id, "oshwdrv.%d.stepenc.%02d.step-max-vel", bus->busnum, id);
		
		if (retval != 0){
			return retval;
		}
		
		se->step_max_vel = 0.0;
		
		// Stepgen actual frequency parameter
		retval = hal_param_float_newf(HAL_RO, &(se->step_freq), comp_id, "oshwdrv.%d.stepenc.%02d.step-freq", bus->busnum, id);
		
		if (retval != 0){
			return retval;
		}
	
		// Stepgen setup time parameter
		retval = hal_param_u32_newf(HAL_RW, &(se->step_setup_time), comp_id, "oshwdrv.%d.stepenc.%02d.step-setup-time", bus->busnum, id);
		
		if (retval != 0){
			return retval;
		}
		/* 10uS default setup time */
		se->step_setup_time = 10000;
		se->last_setup_time = 0;
		
		// Stepgen step puls width
		retval = hal_param_u32_newf(HAL_RW, &(se->step_pulse_width), comp_id, "oshwdrv.%d.stepenc.%02d.step-pulse-width", bus->busnum, id);
		
		if (retval != 0){
			return retval;
		}
		/* 4uS default pulse width */
		se->step_pulse_width = 4000;
		se->last_pulse_width = 0;
			
		// Set the corresponding Stepencoder for the Encoder Index register
		se->master = (id - (id % 8));
		se->master_index = 0x01 << (id % 8);
		
		// Export Encoder HAL pins
		// Encoder raw position
		retval = hal_pin_s32_newf(HAL_OUT, &(se->count), comp_id, "oshwdrv.%d.stepenc.%02d.enc-count", bus->busnum, id);
		
		if (retval != 0){
			return retval;
		}
		
		// Encoder raw delta
		retval = hal_pin_s32_newf(HAL_OUT, &(se->delta), comp_id, "oshwdrv.%d.stepenc.%02d.enc-delta", bus->busnum, id);
		
		if (retval != 0){
			return retval;
		}
		
        // Encoder scale parameter
        retval = hal_param_float_newf(HAL_RW, &(se->enc_scale), comp_id, "oshwdrv.%d.stepenc.%02d.enc-scale", bus->busnum, id);
		
        if (retval != 0){
            return retval;
        }
		
		se->enc_scale = 1.0;
		
        // Encoder position
        retval = hal_pin_float_newf(HAL_OUT, &(se->position), comp_id, "oshwdrv.%d.stepenc.%02d.enc-position", bus->busnum, id);
		
        if (retval != 0){
            return retval;
        }

		// Encoder measured velocity
		retval = hal_pin_float_newf(HAL_OUT, &(se->enc_vel), comp_id, "oshwdrv.%d.stepenc.%02d.enc-velocity", bus->busnum, id);
		
		if (retval != 0){
		  return retval;
		}
		
		// Encoder index pin
		retval = hal_pin_bit_newf(HAL_OUT, &(se->index), comp_id, "oshwdrv.%d.stepenc.%02d.enc-index", bus->busnum, id);
		
		if (retval != 0){
			return retval;
		}
		
		// Encoder index enable Bit
		retval = hal_pin_bit_newf(HAL_IO, &(se->index_enable), comp_id, "oshwdrv.%d.stepenc.%02d.enc-index-enable", bus->busnum, id);
		
		if (retval != 0){
			return retval;
		}
		
		// Init all (clear the encoder counter, stop the stepgen, ...)
		SelWrt(STEPENC_ENC_CLR, (se->wr_addr + STEPENC_CONTROL), port_addr[bus->busnum]);
		SelWrt(0, (se->wr_addr + STEPENC_CONTROL), port_addr[bus->busnum]);
	}
	
	// Extend the EPP read addresses, if needed
	if (bus->read_end_addr < (se->rd_addr + STEPENC_ENC_HIGH + 1)){
		bus->read_end_addr = se->rd_addr + STEPENC_ENC_HIGH + 1; // The first Stepencoder has always an Encoder Index register
	}
	
	bus->read_end_addr += (cnt / 8); // Add a register every 8th Stepencoders
	
	// Extend the EPP write addresses, if needed
	if (bus->write_end_addr < (se->wr_addr + STEPENC_GEN_HI)){
		bus->write_end_addr = se->wr_addr + STEPENC_GEN_HI;
	}
	
	return 0;
}

// Export all Aout to the HAL
static int export_aout (bus_data_t *bus)
{
    int cnt, retval, id, out;
	aout_t *ao;
	
	cnt = count_modules_id(bus, MODULE_ID_AOUT);
    bus->num_aout = cnt;
    rtapi_print_msg(RTAPI_MSG_INFO, "oshwdrv: Exporting Aout %d\n", cnt);
	
	// Return if no module was found
	if (cnt < 1){
		return 0;
	}
	
    // Allocate shared memory
    bus->aout = hal_malloc(cnt * sizeof(aout_t));
    
    if (bus->aout == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "oshwdrv: ERROR: hal_malloc() failed\n");
        return -1;
    }
    
    out = 0;
    
	for (id = 0; id < cnt; id++){
		// Loop through all modules found
		ao = &(bus->aout[id]);
		retval = module_base_addr(bus, MODULE_ID_AOUT, id);
		
		if (retval != 0){
			return retval;
		}
		
		// Export Aout HAL pins
		// Aout enable pin
		retval = hal_pin_bit_newf(HAL_IN, &(ao->enable), comp_id, "oshwdrv.%d.adcout.%02d-%02d.enable", bus->busnum, out, (out + 1));
		
		if (retval != 0){
			return retval;
		}
		
		// Aout value 0 pin
		retval = hal_pin_float_newf(HAL_IN, &(ao->value0), comp_id, "oshwdrv.%d.adcout.%02d.value", bus->busnum, out);
		
		if (retval != 0){
			return retval;
		}
		
		// Aout offset 0 parameter
		retval = hal_param_float_newf(HAL_RW, &(ao->offset0), comp_id, "oshwdrv.%d.adcout.%02d.offset", bus->busnum, out);
		
		if (retval != 0){
			return retval;
		}
		
		// Aout scale 0 parameter
		retval = hal_param_float_newf(HAL_RW, &(ao->scale0), comp_id, "oshwdrv.%d.adcout.%02d.scale", bus->busnum, out);
		
		if (retval != 0){
			return retval;
		}
		
		out++;
		
		// Aout value 1 pin
		retval = hal_pin_float_newf(HAL_IN, &(ao->value1), comp_id, "oshwdrv.%d.adcout.%02d.value", bus->busnum, out);
		
		if (retval != 0){
			return retval;
		}
		
		// Aout offset 1 parameter
		retval = hal_param_float_newf(HAL_RW, &(ao->offset1), comp_id, "oshwdrv.%d.adcout.%02d.offset", bus->busnum, out);
		
		if (retval != 0){
			return retval;
		}
		
		// Aout scale 1 parameter
		retval = hal_param_float_newf(HAL_RW, &(ao->scale1), comp_id, "oshwdrv.%d.adcout.%02d.scale", bus->busnum, out);
		
		if (retval != 0){
			return retval;
		}
	}
	
	// Extend the EPP write addresses, if needed
	if (bus->write_end_addr < (ao->wr_addr + AOUT_HIGH)){
		bus->write_end_addr = ao->wr_addr + AOUT_HIGH;
	}
	
	return 0;
}

// Export all Ain to the HAL
static int export_ain (bus_data_t *bus)
{
    int cnt, retval, id, out;
	ain_t *ai;
	
	cnt = count_modules_id(bus, MODULE_ID_AIN);
    bus->num_ain = cnt;
    rtapi_print_msg(RTAPI_MSG_INFO, "oshwdrv: Exporting Ain %d\n", cnt);
	
	// Return if no module was found
	if (cnt < 1){
		return 0;
	}
	
    // Allocate shared memory
    bus->ain = hal_malloc(cnt * sizeof(ain_t));
    
    if (bus->ain == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "oshwdrv: ERROR: hal_malloc() failed\n");
        return -1;
    }
    
    out = 0;
    
	for (id = 0; id < cnt; id++){
		// Loop through all modules found
		ai = &(bus->ain[id]);
		retval = module_base_addr(bus, MODULE_ID_AIN, id);
		
		if (retval != 0){
			return retval;
		}
		
		// Export Ain HAL pins
		// Ain value 0 pin
		retval = hal_pin_float_newf(HAL_OUT, &(ai->value0), comp_id, "oshwdrv.%d.adcin.%02d.value", bus->busnum, out);
		
		if (retval != 0){
			return retval;
		}
		
		// Ain offset 0 parameter
		retval = hal_param_float_newf(HAL_RW, &(ai->offset0), comp_id, "oshwdrv.%d.adcin.%02d.offset", bus->busnum, out);
		
		if (retval != 0){
			return retval;
		}
		
		// Ain scale 0 parameter
		retval = hal_param_float_newf(HAL_RW, &(ai->scale0), comp_id, "oshwdrv.%d.adcin.%02d.scale", bus->busnum, out);
		
		if (retval != 0){
			return retval;
		}
		
		out++;
		
		// Ain value 1 pin
		retval = hal_pin_float_newf(HAL_OUT, &(ai->value1), comp_id, "oshwdrv.%d.adcin.%02d.value", bus->busnum, out);
		
		if (retval != 0){
			return retval;
		}
		
		// Ain offset 1 parameter
		retval = hal_param_float_newf(HAL_RW, &(ai->offset1), comp_id, "oshwdrv.%d.adcin.%02d.offset", bus->busnum, out);
		
		if (retval != 0){
			return retval;
		}
		
		// Ain scale 1 parameter
		retval = hal_param_float_newf(HAL_RW, &(ai->scale1), comp_id, "oshwdrv.%d.adcin.%02d.scale", bus->busnum, out);
		
		if (retval != 0){
			return retval;
		}
	}
	
	// Extend the EPP write addresses, if needed
	if (bus->read_end_addr < (ai->rd_addr + AIN_HIGH)){
		bus->read_end_addr = ai->rd_addr + AIN_HIGH;
	}
	
	return 0;
}

// Export all PWMo to the HAL
static int export_pwmo (bus_data_t *bus)
{
    int cnt, retval, id;
	pwmo_t *pw;

	cnt = count_modules_id(bus, MODULE_ID_PWMO);
    bus->num_pwmo = cnt;
    rtapi_print_msg(RTAPI_MSG_INFO, "oshwdrv: Exporting PWMo %d\n", cnt);
	
	// Return if no module was found
	if (cnt < 1){
		return 0;
	}
	
    // Allocate shared memory
    bus->pwmo = hal_malloc(cnt * sizeof(pwmo_t));
    
    if (bus->pwmo == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "oshwdrv: ERROR: hal_malloc() failed\n");
        return -1;
    }
    
	for (id = 0; id < cnt; id++){
		// Loop through all modules found
		pw = &(bus->pwmo[id]);
		retval = module_base_addr(bus, MODULE_ID_PWMO, id);
		
		if (retval != 0){
			return retval;
		}
		
		// Export PWMo HAL pins
		// PWMo enable pin
		retval = hal_pin_bit_newf(HAL_IN, &(pw->enable), comp_id, "oshwdrv.%d.pwmout.%02d.enable", bus->busnum, id);
		
		if (retval != 0){
			return retval;
		}
		
		// PWMo value pin
		retval = hal_pin_float_newf(HAL_IN, &(pw->value), comp_id, "oshwdrv.%d.pwmout.%02d.value", bus->busnum, id);
		
		if (retval != 0){
			return retval;
		}
		
		// PWMo offset pin
		retval = hal_param_float_newf(HAL_RW, &(pw->offset), comp_id, "oshwdrv.%d.pwmout.%02d.offset", bus->busnum, id);
		
		if (retval != 0){
			return retval;
		}
		
		// PWMo scale pin
		retval = hal_param_float_newf(HAL_RW, &(pw->scale), comp_id, "oshwdrv.%d.pwmout.%02d.scale", bus->busnum, id);
		
		if (retval != 0){
			return retval;
		}
		
		// PWMo frequency parameter
		retval = hal_param_float_newf(HAL_RW, &(pw->frequency), comp_id, "oshwdrv.%d.pwmout.%02d.frequency", bus->busnum, id);
		
		if (retval != 0){
			return retval;
		}
		
		pw->last_frequency = 0.0;
	}
	
	// Extend the EPP write addresses, if needed
	if (bus->write_end_addr < (pw->wr_addr + PWMO_PULSE_HIGH)){
		bus->write_end_addr = pw->wr_addr + PWMO_PULSE_HIGH;
	}
	
	return 0;
}

// Export all Jog to the HAL
static int export_jog (bus_data_t *bus)
{
    int cnt, retval, id;
	jog_t *jo;

	cnt = count_modules_id(bus, MODULE_ID_JOGENC);
    bus->num_jog = cnt;
    rtapi_print_msg(RTAPI_MSG_INFO, "oshwdrv: Exporting Jog %d\n", cnt);
	
	// Return if no module was found
	if (cnt < 1){
		return 0;
	}
	
    // Allocate shared memory
    bus->jog = hal_malloc(cnt * sizeof(jog_t));
    
    if (bus->jog == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "oshwdrv: ERROR: hal_malloc() failed\n");
        return -1;
    }
    
	for (id = 0; id < cnt; id++){
		// Loop through all modules found
		jo = &(bus->jog[id]);
		retval = module_base_addr(bus, MODULE_ID_JOGENC, id);
		
		if (retval != 0){
			return retval;
		}
		
		// Export Jog-Encoder HAL pins
		// Jog clear pin
		retval = hal_pin_bit_newf(HAL_IN, &(jo->clear), comp_id, "oshwdrv.%d.jog.%02d.clear", bus->busnum, id);
		
		if (retval != 0){
			return retval;
		}
		
		// Jog raw position
		retval = hal_pin_s32_newf(HAL_OUT, &(jo->count), comp_id, "oshwdrv.%d.jog.%02d.count", bus->busnum, id);
		
		if (retval != 0){
			return retval;
		}
		
		// Jog raw delta
		retval = hal_pin_s32_newf(HAL_OUT, &(jo->delta), comp_id, "oshwdrv.%d.jog.%02d.delta", bus->busnum, id);
		
		if (retval != 0){
			return retval;
		}
		
		// Clear encoder counter
		SelWrt(JOG_CLEAR, (jo->wr_addr + JOG_CONTROL), port_addr[bus->busnum]);
		SelWrt(0, (jo->wr_addr + JOG_CONTROL), port_addr[bus->busnum]);
	}
	
	// Extend the EPP read addresses, if needed
	if (bus->read_end_addr < (jo->rd_addr + JOG_COUNT_HIGH)){
		bus->read_end_addr = jo->rd_addr + JOG_COUNT_HIGH;
	}
	
	// Extend the EPP write addresses, if needed
	if (bus->write_end_addr < (jo->wr_addr + JOG_CONTROL)){
		bus->write_end_addr = jo->wr_addr + JOG_CONTROL;
	}
	
	return 0;
}

// Export all Watchdog to the HAL
static int export_watchdog (bus_data_t *bus)
{
    int cnt, retval;
	watchdog_t *wd;

	cnt = count_modules_id(bus, MODULE_ID_WATCHDOG);
    
	// Return if more than one module was found
	if (cnt > 1){
		return -1;
	}
	
    rtapi_print_msg(RTAPI_MSG_INFO, "oshwdrv: Exporting Watchdog %d\n", cnt);
	
	// Return if no module was found
	if (cnt < 1){
		return 0;
	}
	
    // Allocate shared memory
    bus->watchdog = hal_malloc(sizeof(watchdog_t));
    
    if (bus->watchdog == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "oshwdrv: ERROR: hal_malloc() failed\n");
        return -1;
    }
    
	wd = bus->watchdog;
	retval = module_base_addr(bus, MODULE_ID_WATCHDOG, 0);
	
	if (retval != 0){
		return retval;
	}
	
	// Export Watchdog HAL pins
	// Timeout pin
	retval = hal_pin_bit_newf(HAL_OUT, &(wd->timeout), comp_id, "oshwdrv.%d.watchdog.timeout", bus->busnum);
	
	if (retval != 0){
		return retval;
	}
	
	// Estop pin
	retval = hal_pin_bit_newf(HAL_IN, &(wd->estop), comp_id, "oshwdrv.%d.watchdog.estop", bus->busnum);
	
	if (retval != 0){
		return retval;
	}
	
	// Clear Config register
	SelWrt(0, (wd->wr_addr + WATCHDOG_CONFIG), port_addr[bus->busnum]);
	
	// Extend the EPP read addresses, if needed
	if (bus->read_end_addr < (wd->rd_addr + WATCHDOG_STATUS)){
		bus->read_end_addr = wd->rd_addr + WATCHDOG_STATUS;
	}
	
	// Extend the EPP write addresses, if needed
	if (bus->write_end_addr < (wd->wr_addr + WATCHDOG_CONFIG)){
		bus->write_end_addr = wd->wr_addr + WATCHDOG_CONFIG;
	}
	
	return 0;
}

/***********************************************************************
*                         REALTIME I/O FUNCTION                        *
************************************************************************/
// Tests for an EPP bus timeout, and clears it if so
static int ClrTimeout(unsigned int port_addr)
{
    unsigned char r;

    r = rtapi_inb(STATUSPORT(port_addr));
    
    if(!(r & 0x01)){
        return 0;
    }
    
    r = rtapi_inb(STATUSPORT(port_addr));
    rtapi_outb(r | 0x01, STATUSPORT(port_addr)); /* Some reset by writing 1 */
    r = rtapi_inb(STATUSPORT(port_addr));
    
    return !(r & 0x01);
}

// Sets the EPP address and then reads one byte from that address
static unsigned short SelRead(unsigned char epp_addr, unsigned int port_addr, int eppdir)
{
    unsigned char b;
    
    ClrTimeout(port_addr);
    /* set port direction to output */
    rtapi_outb(0x04,CONTROLPORT(port_addr));
    /* write epp address to port */
    rtapi_outb(epp_addr,ADDRPORT(port_addr));
    
    if (eppdir == 1) {
        /* set port direction to input */
        rtapi_outb(0x24,CONTROLPORT(port_addr));
    }
    /* read data value */
    b = rtapi_inb(DATAPORT(port_addr));
    return b;
}

// Reads one byte from EPP, use only after SelRead
static unsigned short ReadMore(unsigned int port_addr)
{
    unsigned char b;
    b = rtapi_inb(DATAPORT(port_addr));
    return b;
}

// Sets the EPP address and then writes one byte to that address
static void SelWrt(unsigned char byte, unsigned char epp_addr, unsigned int port_addr)
{
    ClrTimeout(port_addr);
    /* set port direction to output */
    rtapi_outb(0x04,CONTROLPORT(port_addr));
    /* write epp address to port */
    rtapi_outb(epp_addr,ADDRPORT(port_addr));
    /* write data to port */
    rtapi_outb(byte,DATAPORT(port_addr));
    return;
}

// Writes one byte to EPP, use only after SelWrt
static void WrtMore(unsigned char byte, unsigned int port_addr)
{
    rtapi_outb(byte,DATAPORT(port_addr));
    return;
}
