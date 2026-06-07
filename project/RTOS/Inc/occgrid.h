/**
  ******************************************************************************
  * @file    occgrid.h
  * @brief   2D occupancy grid with int8 log-odds (no float in storage).
  *
  *  World frame: same as EKF/odometry (start state = (0, 0, 0), +x =
  *  robot's initial forward, +y = robot's initial left).
  *
  *  Start cell convention: EKF (0, 0) maps to cell (START_CX, START_CY)
  *  where START_CX = floor(GRID_W * START_FRAC_X) etc. Changing the map
  *  size / start fraction is a single-header recompile — every derived
  *  constant (cell counts, start cell) propagates from the five macros.
  *
  *  Storage: int8_t grid[GRID_H][GRID_W] in BSS.
  *  Update : Bresenham ray cast per ultrasonic; intermediate cells get
  *           L_FREE_DELTA, end cell L_OCC_DELTA (if real hit) or
  *           L_FREE_DELTA (if no echo / max range).
  *  Extract: scan rows/cols for runs of >= WALL_MIN_RUN occupied cells;
  *           push them as wall_seg_t to map_geom for the EKF.
  ******************************************************************************
  */

#ifndef __OCCGRID_H
#define __OCCGRID_H

#include <stdint.h>

/* ===========================================================================
 *  Map / cell geometry — change these 5 macros to retarget the grid.
 * =========================================================================== */
#define MAP_W_CM        500                  /* world width  (x extent) */
#define MAP_H_CM        1000                 /* world height (y extent) */
#define CELL_CM         20                   /* edge length of one cell */
#define START_FRAC_X    (1.0f / 2.0f)        /* start cell x as fraction of GRID_W */
#define START_FRAC_Y    (1.0f / 10.0f)       /* start cell y as fraction of GRID_H */

/* Derived — do not edit. */
#define GRID_W          (MAP_W_CM / CELL_CM)
#define GRID_H          (MAP_H_CM / CELL_CM)
#define START_CX        ((int)(GRID_W * START_FRAC_X))
#define START_CY        ((int)(GRID_H * START_FRAC_Y))

/* ===========================================================================
 *  Log-odds tunables (int8_t saturating arithmetic; no float).
 *    0 = unknown, < 0 = free belief, > 0 = occupied belief.
 *  Asymmetric deltas (occ 2x stronger than free) so a sparse-but-real
 *  obstacle saturates faster than a flaky free reading erodes it.
 * =========================================================================== */
#define L_MIN            (-100)
#define L_MAX             (+100)
#define L_FREE_DELTA      (-2)
#define L_OCC_DELTA       (+4)
#define L_FREE_THRESH     (-20)              /* below: A* treats as confidently free  */
#define L_OCC_THRESH      (+40)              /* at/above: A* impassable, wall extract */

/* Skip a sensor's update on this tick if its 7-sample stddev (cm) is
 * above the threshold — too noisy to trust the median. */
#define OCC_SIGMA_REJECT  3

/* Wall extraction: min consecutive occupied cells (row or column) to
 * form one wall_seg_t. */
#define WALL_MIN_RUN      3

/* Max ray length used when a sensor returns "no echo" — walk that far
 * marking free cells, do not place an occupied endpoint. Matches the
 * EKF's D_OPEN clamp so behavior is consistent. */
#define OCC_RAY_MAX_CM    150

/* ===========================================================================
 *  API
 * =========================================================================== */
void   occ_init(void);
void   occ_update(float x_cm, float y_cm, float theta_rad,
                  int dF, int dL, int dR,
                  int sF, int sL, int sR);
int8_t occ_get(int cx, int cy);              /* L_MIN out of bounds */
void   occ_dump_ascii_uart(void);            /* '.' free, '#' occ, '?' unknown */
void   walls_extract_from_grid(void);        /* replaces map_geom walls[] */

#endif /* __OCCGRID_H */
