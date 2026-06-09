/**
  ******************************************************************************
  * @file    main.h
  * @brief   Shared types and BSP-level macros.
  *          Calibration / tunable constants live at the top of main.c.
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx_hal.h"
#include "stm324x9i_eval.h"
#include <stdbool.h>
#include <stdint.h>

/* ---------- BSP / encoder ISR ---------- */
#define READY                           3


/* ---------- FSM types ----------
 * Simplified 3-state FSM: INIT -> DRIVE -> EMERGENCY.
 * Navigation is heading-priority (N > E/W > S); no wall-following. */
typedef enum {
    INIT = 0,                /* sensor warm-up; wait for valid readings */
    DRIVE,                   /* normal cruise: straight + CORNER + VEER */
    EMERGENCY                /* imminent front collision -> priority pivot */
} DriveState;

/* canProgressDirection() return bitmask */
#define DIR_LEFT     0x01
#define DIR_FORWARD  0x02
#define DIR_RIGHT    0x04

#endif /* __MAIN_H */
