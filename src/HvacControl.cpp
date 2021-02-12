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

void HvacControlClass::toggleRelay(HvacRelay relay, HvacStatLed led, bool on) {
	uint8_t relAddr = this->getRelayAddress(relay);
	uint8_t ledAddr = this->getLedAddress(led);
	this->_busController->digitalWrite(relAddr, on ? HIGH : LOW);
	this->_busController->digitalWrite(ledAddr, on ? HIGH : LOW);
}

void HvacControlClass::setMode(HvacMode mode) {
	if (this->_mode != mode && this->modeChangeHandler != NULL) {
		this->_mode = mode;
		this->modeChangeHandler(mode);
	}
}

void HvacControlClass::fanOn() {
	Serial.println(F("INFO: Fan relay on."));
	this->toggleRelay(HvacRelay::FAN, HvacStatLed::FAN, true);
}

void HvacControlClass::fanOff() {
	Serial.println(F("INFO: Fan relay off."));
	this->toggleRelay(HvacRelay::FAN, HvacStatLed::FAN, false);
}

void HvacControlClass::compressorOn() {
	Serial.println(F("INFO: Compressor relay on."));
	this->toggleRelay(HvacRelay::COMPRESSOR, HvacStatLed::COMPRESSOR, true);
}

void HvacControlClass::compressorOff() {
	Serial.println(F("INFO: Compressor relay off."));
	this->toggleRelay(HvacRelay::COMPRESSOR, HvacStatLed::COMPRESSOR, false);
}

void HvacControlClass::reversingValveOn() {
	Serial.println(F("INFO: Reversing valve relay on."));
	this->toggleRelay(HvacRelay::REVERSINGVALVE, HvacStatLed::REVERSINGVALVE, true);
}

void HvacControlClass::reversingValveOff() {
	Serial.println(F("INFO: Reversing valve relay off."));
	this->toggleRelay(HvacRelay::REVERSINGVALVE, HvacStatLed::REVERSINGVALVE, false);
}

void HvacControlClass::auxHeatOn() {
	Serial.println(F("INFO: Aux heat relay on."));
	this->toggleRelay(HvacRelay::AUX, HvacStatLed::AUX, true);
}

void HvacControlClass::auxHeatOff() {
	Serial.println(F("INFO: Aux heat relay off."));
	this->toggleRelay(HvacRelay::AUX, HvacStatLed::AUX, false);
}

void HvacControlClass::setModeChangeHandler(void (*modeChangeHandler)(HvacMode mode)) {
	this->modeChangeHandler = modeChangeHandler;
}

String HvacControlClass::getModeDescription(HvacMode mode) {
	String result = "unknown";
	switch (mode) {
		case HvacMode::OFF:
			result = "OFF";
			break;
		case HvacMode::COOL:
			result = "COOL";
			break;
		case HvacMode::FANONLY:
			result = "FAN ONLY";
			break;
		case HvacMode::HEAT:
			result = "HEAT";
			break;
		default:
			break;
	}

	return result;
}

bool HvacControlClass::isAuxHeatOn() {
	uint8_t ledAddr = this->getLedAddress(HvacStatLed::AUX);
	return this->_busController->digitalRead(ledAddr) == HIGH;
}

HvacControlClass HvacControl;

