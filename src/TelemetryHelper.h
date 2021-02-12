#ifndef _TELEMETRY_HELPER_H
#define _TELEMETRY_HELPER_H

#include <Arduino.h>

/**
 * Possible system states.
 */
enum class SystemState: uint8_t {
	BOOTING = 0,
	NORMAL = 1,
	DISABLED = 3,
	UPDATING = 4
};

enum class ControlCommand: uint8_t {
	/**
	 * Disable the system. Causes the system to ignore any further incoming
	 * control commands until the "ENABLE" command is received.
	 */
	DISABLE = 0,

	/**
	 * Enables the system if it is currently disabled.
	 */
	ENABLE = 1,

	/**
	 * Reboot the system.
	 */
	REBOOT = 2,

	/**
	 * Get the current status of the system and all controlled outlets.
	 */
	REQUEST_STATUS = 3,

	/**
	 * Reset the onboard I/O controller.
	 */
	IO_RESET = 4,

	SET_MODE = 5,

	SET_TEMP = 6
};

class TelemetryHelper
{
public:
	/**
	 * Gets a description string for the specified MQTT state.
	 * @param  state The state to get the description for.
	 * @return       A description of the specified state.
	 */
	static String getMqttStateDesc(int state);

	/**
	 * Gets a description string of the specified system state.
	 * @param  state The system state to get a description of.
	 * @return       The description of the specified state.
	 */
	static String getSystemStateString(SystemState state);

private:
};

#endif