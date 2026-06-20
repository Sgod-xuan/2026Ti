#ifndef PLATFORM_C07A_PINMAP_H
#define PLATFORM_C07A_PINMAP_H

/*
 * WHEELTEC C07A core board pin map.
 *
 * The C07A onboard user button SW1 is wired to MSPM0G3507 PB8.
 * SW1 is active-low: released reads high, pressed reads low.
 */

#define C07A_START_BUTTON_PORT_NAME "PB"
#define C07A_START_BUTTON_PIN_NUMBER 8u
#define C07A_START_BUTTON_SIGNAL_NAME "PB8"
#define C07A_START_BUTTON_ACTIVE_LOW 1u

#endif
