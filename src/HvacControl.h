#ifndef _HVAC_CONTROL_H
#define _HVAC_CONTROL_H

#include <Arduino.h>
#include "Adafruit_MCP23017.h"

enum class HvacMode : uint8_t {
	OFF = 0,
	COOL = 1,
	HEAT = 2,
	FANONLY = 3
};

enum class HvacRelay : uint8_t {
	COMPRESSOR = 0,
	REVERSINGVALVE = 1,
	FAN = 2,
	AUX = 3
};

enum class HvacStatLed : uint8_t {
	COMPRESSOR = 0,
	REVERSINGVALVE = 1,
	FAN = 2,
	AUX = 3
};

class HvacControlClass
{
public:
	HvacControlClass();
	bool detect();
	void begin(Adafruit_MCP23017 *busController);
	HvacMode getMode();
	void setMode(HvacMode mode);

private:
	uint8_t getRelayAddress(HvacRelay relay);
	uint8_t getLedAddress(HvacStatLed led);
	Adafruit_MCP23017 *_busController;
	HvacMode _mode;
};

extern HvacControlClass HvacControl;

#endif