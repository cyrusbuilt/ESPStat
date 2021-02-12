#ifndef _HVAC_CONTROL_H
#define _HVAC_CONTROL_H

#include <Arduino.h>
#include "Adafruit_MCP23017.h"

#define SETPOINT_MAX 90.0          // Maximum temperature setpoint value (in degrees Fahrenheit).
#define SETPOINT_MIN 50.0          // Minimum temperature setpoint value (in degrees Fharenheit).
#define AUX_HEAT_HYSTERESIS 3.0     // Auxillary heat mode temperature threshold value.
#define COMPRESSOR_HYSTERSIS 1.0   // Compressor activation temperature threshold value.
#define FAN_START_STOP_DELAY 10000
#define COMPRESSOR_LOCKOUT_TIME 1000 * 60 * 5

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
	void fanOn();
	void fanOff();
	void compressorOn();
	void compressorOff();
	void reversingValveOn();
	void reversingValveOff();
	void auxHeatOn();
	void auxHeatOff();
	bool isAuxHeatOn();
	void setModeChangeHandler(void (*modeChangeHandler)(HvacMode mode));
	String getModeDescription(HvacMode mode);

private:
	uint8_t getRelayAddress(HvacRelay relay);
	uint8_t getLedAddress(HvacStatLed led);
	void toggleRelay(HvacRelay relay, HvacStatLed led, bool on);
	Adafruit_MCP23017 *_busController;
	HvacMode _mode;
	void (*modeChangeHandler)(HvacMode mode);
};

extern HvacControlClass HvacControl;

#endif