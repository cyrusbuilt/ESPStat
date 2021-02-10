#ifndef _PINMAP_H
#define _PINMAP_H

// Pin definitions on the MCP23017
#define GPA0 0
#define GPA1 1
#define GPA2 2
#define GPA3 3
#define GPA4 4
#define GPA5 5
#define GPA6 6
#define GPA7 7
#define GPB0 8
#define GPB1 9
#define GPB2 10
#define GPB3 11
#define GPB4 12
#define GPB5 13
#define GPB6 14
#define GPB7 15

// Relays
#define RELAY_COMPRESSOR GPA7
#define RELAY_FAN GPA6
#define RELAY_REV_VALVE GPA5
#define RELAY_AUX_HEAT GPA4

// LEDs
#define LED_COMPRESSOR GPB0
#define LED_FAN GPB1
#define LED_REV_VALVE GPB2
#define LED_AUX_HEAT GPB3

// Detection
#define DET_IN GPB7
#define DET_OUT GPA0

#endif