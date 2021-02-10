#include "HvacControl.h"
#include "pinmap.h"

HvacControlClass::HvacControlClass() {}

uint8_t HvacControlClass::getRelayAddress(HvacRelay relay) {
	uint8_t result = -1;
	switch (relay) {
		case HvacRelay::AUX:
			result = RELAY_AUX_HEAT;
			break;
		case HvacRelay::COMPRESSOR:
			result = RELAY_COMPRESSOR;
			break;
		case HvacRelay::FAN:
			result = RELAY_FAN;
			break;
		case HvacRelay::REVERSINGVALVE:
			result = RELAY_REV_VALVE;
			break;
		default:
			break;
	}

	return result;
}

uint8_t HvacControlClass::getLedAddress(HvacStatLed led) {
	uint8_t result = -1;
	switch (led) {
		case HvacStatLed::AUX:
			result = LED_AUX_HEAT;
			break;
		case HvacStatLed::COMPRESSOR:
			result = LED_COMPRESSOR;
			break;
		case HvacStatLed::FAN:
			result = LED_FAN;
			break;
		case HvacStatLed::REVERSINGVALVE:
			result = LED_REV_VALVE;
			break;
	}

	return result;
}

void HvacControlClass::begin(Adafruit_MCP23017 *busController) {
	this->_busController = busController;
	this->_mode = HvacMode::OFF;
	for (uint8_t i = 0; i < 4; i++) {
		uint8_t relAddr = this->getRelayAddress((HvacRelay)i);
		this->_busController->pinMode(relAddr, OUTPUT);
		this->_busController->digitalWrite(relAddr, LOW);

		uint8_t ledAddr = this->getLedAddress((HvacStatLed)i);
		this->_busController->pinMode(ledAddr, OUTPUT);
		this->_busController->digitalWrite(ledAddr, LOW);
	}
}

bool HvacControlClass::detect() {
	this->_busController->pinMode(DET_IN, INPUT);
	this->_busController->pinMode(DET_OUT, OUTPUT);
	this->_busController->digitalWrite(DET_OUT, HIGH);
	
	uint8_t result = _busController->digitalRead(DET_IN);
	this->_busController->digitalWrite(DET_OUT, LOW);
	return result == HIGH;
}

HvacMode HvacControlClass::getMode() {
	return this->_mode;
}

HvacControlClass HvacControl;

