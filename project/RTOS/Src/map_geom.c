/**
  ******************************************************************************
  * @file    map_geom.c
  * @brief   Static wall-segment storage shared by EKF and (later) A* planner.
  ******************************************************************************
  */

#include "map_geom.h"

wall_seg_t walls[MAX_WALLS];
uint8_t    n_walls = 0;

void walls_clear(void)
{
    n_walls = 0;
}

bool walls_add(float x0, float y0, float x1, float y1)
{
    if (n_walls >= MAX_WALLS) return false;
    walls[n_walls].x0 = x0;
    walls[n_walls].y0 = y0;
    walls[n_walls].x1 = x1;
    walls[n_walls].y1 = y1;
    n_walls++;
    return true;
}
