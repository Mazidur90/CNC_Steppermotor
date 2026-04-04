#pragma once

#include <Arduino.h>
struct can_frame;

bool masterSendCanFrame(struct can_frame* frame);
void masterSendHeartbeat();
void masterSendMotorCmd(uint8_t dir, uint16_t steps, uint16_t rpm, uint8_t accelFactor);
void masterSendStopCmd(uint8_t stopType);
void masterSendHomeCmd(uint8_t dir, uint8_t speedPct);
void masterSendJog(uint8_t dir, uint8_t speedPct, uint8_t enable);
void masterSendDigitalOut(uint8_t mask, uint8_t value);
void masterSendConfigGet(uint8_t key);
void masterSendConfigSet(uint8_t key, uint32_t value);
