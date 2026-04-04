#pragma once

void slavePeripheralsInitPins();
void slaveApplyDoutPins();
bool slaveLimitMinTripped();
bool slaveLimitMaxTripped();
void slavePollMcpErrors();
void slaveUpdateStatusLed();
