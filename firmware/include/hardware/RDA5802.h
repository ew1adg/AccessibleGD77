#if defined(PLATFORM_RD5R)
#ifndef _OPENGD77_RDA5802_H_
#define _OPENGD77_RDA5802_H_

#include <stdint.h>
#include "interfaces/i2c.h"

/* Chip addresses */
#define RDA5802_BATCH_ADDRESS 0x10
#define RDA5802_ADDRESS	0x11

/* Registers */
#define RDA5802_CHIP_ID_REG 0x00

/* Registers */
// 0x02H
#define RDA5802_DHIZ                0b10000000 // Audio out enable (1) / High impedance (0)
#define RDA5802_DMUTE               0b01000000 // Audio unmuted (1) / muted (0)
#define RDA5802_MONO                0b00100000 // Mono mode (1) / stereo mode (0)
#define RDA5802_BASS                0b00010000 // Bass boost (1)
#define RDA5802_RCLK_NON_CAL_MODE   0b00001000 // RCLK always on (0)
#define RDA5802_RCLK_DIR_IN_MODE    0b00000100 // RCLK direct input mode (1)
#define RDA5802_SEEKUP              0b00000010 // Seek up (0) / down (1)
#define RDA580X_SEEK                0b00000001 // Stop seek (0) / start seek in SEEKUP direction (1)

// 0x02L
#define RDA5802_CLK_MODE_32768      0b00000000 // Suartz frequency
#define RDA5802_NEW_METHOD          0b00000100 // New demodulation method (1)
#define RDA5802_SOFT_RESET          0b00000010 // Reset settings (1)
#define RDA5802_ENABLE              0b00000001 // Power on radio (1)

// 0x03H
#define RDA5802_CHAN_9_2            0b11111111 // CHAN 9..2 bits

// 0x03L
// CHAN 1.. 0 bits
#define RDA5802_CHAN_1_0            0b1100000  // CHAN 1..0 bits
#define RDA5802_TUNE                0b00010000 // Tune enable (1)
#define RDA5802_BAND                0b00001100 // Tuner band selection
#define RDA5802_BAND_US_EUROPE      0b00000000 // 87 - 108 MHz
#define RDA5802_BAND_JAPAN          0b00000100 // 76 - 97 MHz
#define RDA5802_BAND_WORLDWIDE      0b00001000 // 76 - 108 MHz
#define RDA5802_BAND_EASTEUROPE     0b00001100 // 65 - 76 MHz
#define RDA5802_STEP                0b00000011 // Frequency step
#define RDA5802_STEP_100            0b00000000 // 100 KHz step
#define RDA5802_STEP_200            0b00000001 // 200 kHz step
#define RDA5802_STEP_50             0b00000010 // 50 KHz step
#define RDA5802_SPACE_25            0b00000011 // 25 KHz step

// 0x04H
#define RDA5802_DE                  0b00001000 // De-emphasis 75us (0) / 50us (1)
#define RDA5802_SOFTMUTE_EN         0b00000010 // Softmute enable (1)
#define RDA5802_AFCD                0b00000001 // AFC disable (1)

// 0x05H
#define RDA5802_SEEKTH              0b00001111 // Seek SNR threshold, 4bits, default 1000=32dB

// 0x05L
#define RDA5802_LNA_PORT_SEL        0b11000000 // LNA input port selection bit
#define RDA5802_LNA_PORT_SEL_NO     0b00000000 // no input
#define RDA5802_LNA_PORT_SEL_LNAN   0b01000000 // LNAN
#define RDA5802_LNA_PORT_SEL_LNAP   0b10000000 // LNAP (FMIN on RDA5807_FP)
#define RDA5802_LNA_PORT_SEL_DUAL   0b11000000 // Dual port input
#define RDA5802_LNA_ICSEL_BIT       0b00110000 // LNA working current bit
#define RDA5802_LNA_ICSEL_BIT_1_8   0b00000000 // 1.8mA
#define RDA5802_LNA_ICSEL_BIT_2_1   0b00010000 // 2.1mA
#define RDA5802_LNA_ICSEL_BIT_2_5   0b00100000 // 2.5mA
#define RDA5802_LNA_ICSEL_BIT_3_0   0b00110000 // 3.0mA
#define RDA5802_VOLUME              0b00001111 // 4 bits volume (0000 - muted, 1111 - max)

/* Prototypes */
bool initialize_rda5802();
void enable_rda5802();
void disable_rda5802();
void start_seek(bool seekup);
void stop_seek();
void set_freq_rda5802(uint16_t freq);
uint16_t get_freq_rda5802();
uint16_t get_rssi_rda5802();
bool get_stc_flag();
bool get_sf_flag();
status_t RDA5802ReadReg2byte(uint8_t reg, uint16_t *val);
status_t RDA5802BatchWrite(uint8_t *registers);
status_t RDA5802BatchRead(uint8_t *registers);

#endif
#endif
