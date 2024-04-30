#ifndef APP_I2C_H
#define APP_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes
#include "driver/i2c.h"

// Pin Defines
#define I2C_SDA_PIN             GPIO_NUM_21
#define I2C_SCL_PIN             GPIO_NUM_22
// Left AD5272 digipot tristate addr. input pin floating (configurable to otherwise in HW)
// 7 bit slave addr for AD5272 is therefore 0101110 (0x2E)

// Settings
#define I2C_MASTER_NUM                  0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ              400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE       0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE       0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS           1000
#define AD5272_ADDR                     0x2E                       /* Slave address of the AD5272 digital potentiometer */
#define AD5272_TRANSACTION_SIZE_BYTES   2
#define AD5272_RDAC_MAX                 1023
#define AD5272_RDAC_MID                 511
#define AD5272_RDAC_MIN                 0

// Macros
#define AD5272_CNT_TO_OHM(UPR8, LWR8) (((UPR8 << 8 | LWR8) / 1023.0) * 50000 + 50) /* Using 50kOhm variant. Nominal minimum wiper resistance is in range 35-70 ohms, so this is added to the value */
#define PCT_TO_INV_CNT(PCT)            (AD5272_RDAC_MAX - (PCT * AD5272_RDAC_MAX) / 100)

// Typedefs
// AD5272 Commands
typedef enum {
     AD5272_NOP,        /* NOP */
     AD5272_RDAC_WRITE, /* RDAC register write (10 bit data for 1024 wiper positions) */
     AD5272_RDAC_READ,  /* RDAC register read */
     AD5272_50TP_WRITE, /* Write the current RDAC value to the next value in 50TP memory */
     AD5272_SW_RESET,   /* Perform a software reset (sets RDAC value to latest 50TP value. Wiper freezes to midscale if 50-TP memory has not been previously programmed)*/
     AD5272_50TP_READ,  /* Get value at specified 50TP memroy address (0x01 - 0x32) */
     AD5272_50TP_ADDR,  /* Get current address of 50TP memory (incremeted upon each write) */
     AD5272_CTRL_WRITE, /* Control register write */
     AD5272_CTRL_READ,  /* Control register read */
     AD5272_SW_SHUTDOWN /* Perform a software shutdown of the device */
} AD5272_commands_t;

// AD5272 Control register bits
typedef enum {
     AD5272_50TP_WEN,     /* Write enable for 50TP memory */
     AD5272_RDAC_WEN,     /* Write enable for RDAC/wiper register */
     AD5272_RESP_EN,      /* Resistor performance mode enable */
     AD5272_50TP_SUCCESS  /* Status of 50TP memory programming */
} AD5272_ctrl_reg_bits_t;

typedef enum {
     HOLD,          /* Hold the digipot counts where they are*/
     INCREMENT,     /* Increment the existing counts */
     DECREMENT,     /* Decrement the existing counts */
     WIPER_READ,    /* WIPER write is implied by opting into increment or decrement */
     CTRL_READ,
     CTRL_WRITE,
     OTP_READ,
     OTP_ADDR,
     OTP_WRITE,
     RESET, 
     SHUTDOWN          
} AD5272_actions_t;

//typedef AD5272_actions_t uint8_t;

typedef struct {
     uint16_t * rdacReg;             /* A value between 0 and 1024 representing the wiper pos (tap) of the digipot */
     uint16_t * ctrlReg;             /* Hex representation of contents in control reg */
     uint16_t * otpReg;              /* Hex representation of contents in 50tp memory */
     uint16_t * otpAddr;        
} digipot_status_t;

typedef struct {
     AD5272_actions_t * action;      // Hold, increment, decrement (coming soon: excrement)
     uint16_t * wiperValue;                 /* Desired update value; should be between 0 and 1024 */
} digipot_ctrl_t;

typedef struct {
     // NOTE: Other driver-specific configuration info is contained within the app_init function
     char * TAG;
     digipot_status_t * status;    // Digipot status contains all register values
     digipot_ctrl_t * ctrl;        // Action to be performed and value to work with
     bool * updateFlag;
     uint16_t * val;
     int delay_ms;
} i2cMasterParams_t;

// User functions
esp_err_t app_i2c_master_init(void);
//esp_err_t ad5272_write(uint8_t buff[2], uint8_t code, uint16_t data);
esp_err_t ad5272_ctrl_reg_write(uint8_t code); // Use a bitwise OR'd combination of desired control bit levels
esp_err_t ad5272_ctrl_reg_read(uint8_t read_buff[2]);
esp_err_t ad5272_rdac_reg_write(uint16_t value);
esp_err_t ad5272_rdac_reg_read(uint8_t read_buff[2]);
esp_err_t ad5272_50tp_mem_write(void); // Addressing is automatically handled in the IC. Value comes from current value in RDAC
esp_err_t ad5272_50tp_mem_read(uint8_t addr, uint8_t read_buff[2]);
esp_err_t ad5272_sw_reset(void);
esp_err_t ad5272_sw_shutdown(void);


#ifdef __cplusplus
}
#endif

#endif  // APP_I2C_H



/*
Notes on AD5272


Command 3 in Table 12 programs the contents of the RDAC register to memory.
The first address to be programmed is Location 0x01, see Table
15, and the AD5272/AD5274 increment the 50-TP memory address
for each subsequent program until the memory is full

blah blah programming takes some time and consumes afair bit of current (4ma), LOCKS THE REGISTER, and requires a 1uF ext cap!

Prior to 50-TP activation, the AD5272/AD5274 is preset to midscale
on power-up. It is possible to read back the contents of any of
the 50-TP memory registers through the I2C interface by using
Command 5 in Table 12. The lower six LSB bits, D0 to D5 of the
data byte, select which memory location is to be read back. A
binary encoded version address of the most recently programmed
wiper memory location can be read back using Command 6 in
Table 12. This can be used to monitor the spare memory status of
the 50-TP memory block


AD5272 OPCODES

DC == "DONT CARE"
All cycles involve 2 bytes R/W

0x00 NOP
0x01 << 2 | 10 BIT DATA      - Write contnets of serial register data to RDAC
0x02 << 2 | DC               - Read contents fo the RDAC wiper register
0x03 << 2 | DC               - Store current RDAC setting to next address in 50-TP memory block
0x04 << 2 | DC               - SW Reset; load RDAC with most recently programmed 50-TP value
0x05 << 2 | 5 BIT 50-TP ADDR - Read contents of 50-TP from the specified SDA output in the next frame
0x06 << 2 | DC               - Read contnents of the last 50-TP programmed memory location
0x07 << 2 | 3 BIT DATA       - Write contents of the serial register data to the control register
0x08 << 2 | DC               - Read contents of the control reg CTRL[10:0] = ["000000"&C3|C2|C1|C0]
0x09 << 2 | 1 BIT DATA       - Software shutdown depending on level of the 1 data bit (shutdown if bit == 1)


RDAC CONTROL REGISTER BIT DESCRIPTIONS


C0 - 50-TP PROGRAM ENABLE
     0 - disabled (default)
     1 - enabled
C1 - RDAC REG WRITE PROTECT
     0 - wiper pos is frozen to value in 50-TP memory (default)
     1 - allow update of wiper position
C2 - RESISTOR PERFORMANCE ENABLE
     0 - RDAC resistor tolerance calibration enabled (default)
     1 - disabled
C3 - 50-TP MEMORY PROGRAM SUCCESS BIT
     0 - fuse program command unsuccessful (default)
     1 - fuse program successful
*/