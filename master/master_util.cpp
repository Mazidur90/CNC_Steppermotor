#include "master_util.h"

uint8_t masterRpmToU8(uint16_t rpm) {
    if (rpm > 255u) {
        return 255u;
    }
    return (uint8_t)rpm;
}
