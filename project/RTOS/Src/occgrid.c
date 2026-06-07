/**
  ******************************************************************************
  * @file    occgrid.c
  * @brief   2D occupancy grid + wall extractor — see occgrid.h for layout.
  *
  *  Per ultrasonic beam (called from SensorTask after ekf_update):
  *    1. transform mount offset by current pose -> sensor world origin
  *    2. compute beam end = origin + range * (cos(beam), sin(beam))
  *       where range = clamp(z_meas, OCC_RAY_MAX_CM); hit = z_meas in
  *       (0, OCC_RAY_MAX_CM).
  *    3. Bresenham(origin_cell -> end_cell):
  *         intermediate cells:  saturating += L_FREE_DELTA
  *         end cell + hit:      saturating += L_OCC_DELTA
  *         end cell + no hit:   saturating += L_FREE_DELTA
  *
  *  Wall extraction (called periodically): clears walls[] then re-scans the
  *  grid for runs of >= WALL_MIN_RUN consecutive cells with l >= L_OCC_THRESH
  *  along rows and columns, pushing each run as one wall_seg_t in cm.
  ******************************************************************************
  */

#include "occgrid.h"
#include "map_geom.h"
#include "ekf.h"          /* US_*_DX/DY/PHI mount offsets */
#include <math.h>
#include <stdio.h>

static int8_t grid[GRID_H][GRID_W];

/* -------- internal helpers -------- */
static int8_t sat_add(int8_t v, int delta)
{
    int n = (int)v + delta;
    if (n > L_MAX) n = L_MAX;
    if (n < L_MIN) n = L_MIN;
    return (int8_t)n;
}

static void world_to_cell(float x_cm, float y_cm, int *cx, int *cy)
{
    *cx = (int)floorf(x_cm / (float)CELL_CM) + START_CX;
    *cy = (int)floorf(y_cm / (float)CELL_CM) + START_CY;
}

static int in_bounds(int cx, int cy)
{
    return (cx >= 0 && cx < GRID_W && cy >= 0 && cy < GRID_H);
}

static void cell_apply(int cx, int cy, int delta)
{
    if (!in_bounds(cx, cy)) return;
    grid[cy][cx] = sat_add(grid[cy][cx], delta);
}

/* Cell-center world coordinates (cm). Used by walls_extract to convert
 * a cell index back into a line-segment endpoint. */
static float cell_to_x(int cx) { return ((float)(cx - START_CX) + 0.5f) * (float)CELL_CM; }
static float cell_to_y(int cy) { return ((float)(cy - START_CY) + 0.5f) * (float)CELL_CM; }

/* Bresenham line from (x0,y0) to (x1,y1). For every cell on the path
 * except the endpoint, apply L_FREE_DELTA. The caller handles the
 * endpoint separately (occ vs free). */
static void bresenham_free(int x0, int y0, int x1, int y1)
{
    int dx =  (x1 >= x0) ? (x1 - x0) : (x0 - x1);
    int dy = -((y1 >= y0) ? (y1 - y0) : (y0 - y1));
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    int cx = x0, cy = y0;
    while (cx != x1 || cy != y1) {
        cell_apply(cx, cy, L_FREE_DELTA);
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; cx += sx; }
        if (e2 <= dx) { err += dx; cy += sy; }
    }
    /* endpoint left for caller */
}

/* -------- per-sensor ray update -------- */
static void ray_update(float px, float py, float th,
                       float dx_r, float dy_r, float phi_s,
                       int z_meas, int sigma)
{
    if (sigma > OCC_SIGMA_REJECT) return;

    /* Sensor world origin (mount + body rotation). */
    const float c = cosf(th), s = sinf(th);
    const float ox = px + dx_r * c - dy_r * s;
    const float oy = py + dx_r * s + dy_r * c;
    const float beam = th + phi_s;
    const float bx = cosf(beam), by = sinf(beam);

    /* Range to mark: clamp to OCC_RAY_MAX_CM. hit only when sensor saw
     * something in (0, OCC_RAY_MAX_CM); z==0 means no echo. */
    int range = z_meas;
    int hit   = (z_meas > 0 && z_meas < OCC_RAY_MAX_CM);
    if (range <= 0 || range > OCC_RAY_MAX_CM) range = OCC_RAY_MAX_CM;

    const float ex = ox + (float)range * bx;
    const float ey = oy + (float)range * by;

    int c0x, c0y, c1x, c1y;
    world_to_cell(ox, oy, &c0x, &c0y);
    world_to_cell(ex, ey, &c1x, &c1y);

    bresenham_free(c0x, c0y, c1x, c1y);
    cell_apply(c1x, c1y, hit ? L_OCC_DELTA : L_FREE_DELTA);
}

/* ===========================================================================
 *  Public API
 * =========================================================================== */
void occ_init(void)
{
    for (int y = 0; y < GRID_H; y++)
        for (int x = 0; x < GRID_W; x++)
            grid[y][x] = 0;
}

void occ_update(float x_cm, float y_cm, float theta_rad,
                int dF, int dL, int dR,
                int sF, int sL, int sR)
{
    ray_update(x_cm, y_cm, theta_rad, US_F_DX, US_F_DY, US_F_PHI, dF, sF);
    ray_update(x_cm, y_cm, theta_rad, US_L_DX, US_L_DY, US_L_PHI, dL, sL);
    ray_update(x_cm, y_cm, theta_rad, US_R_DX, US_R_DY, US_R_PHI, dR, sR);
}

int8_t occ_get(int cx, int cy)
{
    if (!in_bounds(cx, cy)) return L_MIN;
    return grid[cy][cx];
}

void occ_dump_ascii_uart(void)
{
    printf("\r\n>> OCC GRID dump (%d x %d, cell=%dcm, start=(%d,%d))",
           GRID_W, GRID_H, CELL_CM, START_CX, START_CY);
    /* Print y high-to-low so the first row visually corresponds to the
     * top of the world map; '*' marks the start cell for orientation. */
    for (int y = GRID_H - 1; y >= 0; y--) {
        printf("\r\n");
        for (int x = 0; x < GRID_W; x++) {
            char ch;
            int8_t v = grid[y][x];
            if (x == START_CX && y == START_CY) ch = '*';
            else if (v >= L_OCC_THRESH)         ch = '#';
            else if (v <= L_FREE_THRESH)        ch = '.';
            else                                ch = '?';
            putchar(ch);
        }
    }
    printf("\r\n");
}

/* Scan one row (or column) for runs of occupied cells; emit a wall
 * segment per run. Returns the number of walls pushed. Stops early if
 * walls_add() rejects (MAX_WALLS reached). */
static int scan_runs_row(int y)
{
    int pushed = 0;
    int run_start = -1;
    for (int x = 0; x <= GRID_W; x++) {
        int occ = (x < GRID_W) && (grid[y][x] >= L_OCC_THRESH);
        if (occ) {
            if (run_start < 0) run_start = x;
        } else if (run_start >= 0) {
            int run_end = x - 1;
            if (run_end - run_start + 1 >= WALL_MIN_RUN) {
                float wx0 = cell_to_x(run_start);
                float wx1 = cell_to_x(run_end);
                float wy  = cell_to_y(y);
                if (!walls_add(wx0, wy, wx1, wy)) return pushed;
                pushed++;
            }
            run_start = -1;
        }
    }
    return pushed;
}

static int scan_runs_col(int x)
{
    int pushed = 0;
    int run_start = -1;
    for (int y = 0; y <= GRID_H; y++) {
        int occ = (y < GRID_H) && (grid[y][x] >= L_OCC_THRESH);
        if (occ) {
            if (run_start < 0) run_start = y;
        } else if (run_start >= 0) {
            int run_end = y - 1;
            if (run_end - run_start + 1 >= WALL_MIN_RUN) {
                float wy0 = cell_to_y(run_start);
                float wy1 = cell_to_y(run_end);
                float wx  = cell_to_x(x);
                if (!walls_add(wx, wy0, wx, wy1)) return pushed;
                pushed++;
            }
            run_start = -1;
        }
    }
    return pushed;
}

void walls_extract_from_grid(void)
{
    walls_clear();
    int total = 0;
    for (int y = 0; y < GRID_H; y++) total += scan_runs_row(y);
    for (int x = 0; x < GRID_W; x++) total += scan_runs_col(x);
    printf("\r\n>> WALLS_EXTRACT n=%d", total);
}
