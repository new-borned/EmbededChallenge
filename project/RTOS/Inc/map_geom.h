/**
  ******************************************************************************
  * @file    map_geom.h
  * @brief   World-frame wall segments used by the EKF measurement model.
  *
  *  A wall is a 2D line segment {(x0,y0) -> (x1,y1)} in cm, in the same
  *  frame as the EKF/odometry pose (start state = (0,0,0); +x = robot's
  *  initial forward, +y = robot's initial left, CCW positive).
  *
  *  Storage is BSS — no allocation. Phase 3's occupancy mapping will later
  *  feed walls via walls_extract_from_grid(); for now the user (or a CALIB
  *  mode) seeds them manually with walls_add().
  ******************************************************************************
  */

#ifndef __MAP_GEOM_H
#define __MAP_GEOM_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_WALLS  64

typedef struct {
    float x0, y0;
    float x1, y1;
} wall_seg_t;

extern wall_seg_t walls[MAX_WALLS];
extern uint8_t    n_walls;

void walls_clear(void);
bool walls_add(float x0, float y0, float x1, float y1);   /* false if full */

#endif /* __MAP_GEOM_H */
