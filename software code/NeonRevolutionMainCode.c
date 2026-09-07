// Lab9Main.c
// Runs on MSPM0G3507
// Lab 9 ECE319K
// Last Modified: May 18, 2026

#include <stdio.h>
#include <stdint.h>
#include <ti/devices/msp/msp.h>
#include "../inc/ST7735.h"
#include "../inc/Clock.h"
#include "../inc/LaunchPad.h"
#include "../inc/TExaS.h"
#include "../inc/Timer.h"
#include "../inc/ADC1.h"
#include "../inc/ADC.h"
#include "../inc/DAC5.h"
#include "SmallFont.h"
#include "LED.h"
#include "Switch.h"
#include "Sound.h"
#include "images/images.h"
#include "../inc/Japan.h"
#include "diskio.h"
#include "integer.h"
#include "JoyStick.h"
#include <math.h>

// ============================================================================
// --- Color Definitions (16-bit BGR565 Format) ---
// ============================================================================
#define NEON_CYAN    0x07FF
#define NEON_PINK    0xF81F
#define NEON_GREEN   0x07E0
#define NEON_YELLOW  0xFFE0
#define BLACK        0x0000
#define ST7735_WHITE 0xFFFF

// ============================================================================
// --- Fixed-Point Raycaster Constants ---
// ============================================================================
#define MOVE_SPEED   6553   // Player movement step size per frame in 16.16 (~0.1)
#define DRONE_SPEED  1500   // Smoothed stalk speed (~0.022) to prevent clipping issues
#define COS_VAL      65203  // cos(0.1 radians) scaled to 16.16 (~0.995)
#define SIN_VAL      6542   // sin(0.1 radians) scaled to 16.16 (~0.0998)

// ============================================================================
// --- Fixed-Point Math Macros (16.16 Signed Fixed-Point Format) ---
// ============================================================================
#define FP_SHIFT     16
#define FLOAT_TO_FP(x) ((int32_t)((x) * (1 << FP_SHIFT)))
#define FP_MUL(a, b)   ((int32_t)(((int64_t)(a) * (b)) >> FP_SHIFT))
#define FP_DIV(a, b)   ((int32_t)(((int64_t)(a) << FP_SHIFT) / (b)))
#define INT_TO_FP(x)   ((x) << FP_SHIFT)
#define FP_TO_INT(x)   ((x) >> FP_SHIFT)

// ============================================================================
// --- Map & Screen Boundaries ---
// ============================================================================
#define MAP_W 48     
#define MAP_H 48     
#define SCR_W 128    
#define SCR_H 160    

// ============================================================================
// --- Frame and Depth Buffers ---
// ============================================================================
static uint16_t screenBuf[64 * 160]; 
static int32_t ZBuffer[SCR_W];        

// ============================================================================
// --- Global Chunk Partition Boundaries ---
// ============================================================================
static int16_t currentChunkXStart = 0; 
static int16_t currentChunkXEnd = 64;   

// ============================================================================ // --- Local Font Array for Safe Buffer Printing --- // ============================================================================ 
static const uint8_t LocalFontTable[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5f, 0x00, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14, 0x24, 0x2a, 0x7f, 0x2a, 0x12, 0x23, 0x13, 0x08, 0x64, 0x62, 0x36, 0x49, 0x55, 0x22, 0x50, 0x00, 0x05, 0x03, 0x00, 0x00, 0x00, 0x1c, 0x22, 0x41, 0x00, 0x00, 0x41, 0x22, 0x1c, 0x00, 0x14, 0x08, 0x3e, 0x08, 0x14, 0x08, 0x08, 0x3e, 0x08, 0x08, 0x00, 0x50, 0x30, 0x00, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x60, 0x60, 0x00, 0x00, 0x20, 0x10, 0x08, 0x04, 0x02, 0x3e, 0x51, 0x49, 0x45, 0x3e, 0x00, 0x42, 0x7f, 0x40, 0x00, 0x42, 0x61, 0x51, 0x49, 0x46, 0x21, 0x41, 0x45, 0x4b, 0x31, 0x18, 0x14, 0x12, 0x7f, 0x10, 0x27, 0x45, 0x45, 0x45, 0x39, 0x3c, 0x4a, 0x49, 0x49, 0x30, 0x01, 0x71, 0x09, 0x05, 0x03, 0x36, 0x49, 0x49, 0x49, 0x36, 0x06, 0x49, 0x49, 0x29, 0x1e, 0x00, 0x36, 0x36, 0x00, 0x00, 0x00, 0x56, 0x36, 0x00, 0x00, 0x08, 0x14, 0x22, 0x41, 0x00, 0x14, 0x14, 0x14, 0x14, 0x14, 0x00, 0x41, 0x22, 0x14, 0x08, 0x02, 0x01, 0x51, 0x09, 0x06, 0x32, 0x49, 0x79, 0x41, 0x3e, 0x7e, 0x11, 0x11, 0x11, 0x7e, 0x7f, 0x49, 0x49, 0x49, 0x36, 0x3e, 0x41, 0x41, 0x41, 0x22, 0x7f, 0x41, 0x41, 0x22, 0x1c, 0x7f, 0x49, 0x49, 0x49, 0x41, 0x7f, 0x09, 0x09, 0x09, 0x01, 0x3e, 0x41, 0x49, 0x49, 0x7a, 0x7f, 0x08, 0x08, 0x08, 0x7f, 0x00, 0x41, 0x7f, 0x41, 0x00, 0x20, 0x40, 0x41, 0x3f, 0x01, 0x7f, 0x08, 0x14, 0x22, 0x41, 0x7f, 0x40, 0x40, 0x40, 0x40, 0x7f, 0x02, 0x0c, 0x02, 0x7f, 0x7f, 0x04, 0x08, 0x10, 0x7f, 0x3e, 0x41, 0x41, 0x41, 0x3e, 0x7f, 0x09, 0x09, 0x09, 0x06, 0x3e, 0x41, 0x51, 0x21, 0x5e, 0x7f, 0x09, 0x19, 0x29, 0x46, 0x46, 0x49, 0x49, 0x49, 0x31, 0x01, 0x01, 0x7f, 0x01, 0x01, 0x3f, 0x40, 0x40, 0x40, 0x3f, 0x1f, 0x20, 0x40, 0x20, 0x1f, 0x3f, 0x40, 0x38, 0x40, 0x3f, 0x63, 0x14, 0x08, 0x14, 0x63, 0x07, 0x08, 0x70, 0x08, 0x07, 0x61, 0x51, 0x49, 0x45, 0x43 }; 

// ============================================================================ // --- 48x48 SURVIVAL MAP --- // ============================================================================ 
const uint8_t worldMap[MAP_H][MAP_W] = {
/* row  0 */ {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
/* row  1 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row  2 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row  3 */ {1,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,1},
/* row  4 */ {1,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,1},
/* row  5 */ {1,0,0,1,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,1,0,0,1},
/* row  6 */ {1,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,1},
/* row  7 */ {1,0,0,1,1,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,0,0,1},
/* row  8 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row  9 */ {1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 10 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 11 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 12 */ {1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1},
/* row 13 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 14 */ {1,1,1,1,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,1,1},
/* row 15 */ {1,1,1,1,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,1,1},
/* row 16 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 17 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 18 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 19 */ {1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1},
/* row 20 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 21 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 22 */ {1,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,1},
/* row 23 */ {1,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,1},
/* row 24 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 25 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 26 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 27 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 28 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 29 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 30 */ {1,1,1,1,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,1,1},
/* row 31 */ {1,1,1,1,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,1,1},
/* row 32 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 33 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 34 */ {1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1},
/* row 35 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 36 */ {1,0,0,1,1,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,0,0,1},
/* row 37 */ {1,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,1},
/* row 38 */ {1,0,0,1,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,1,0,0,1},
/* row 39 */ {1,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,1},
/* row 40 */ {1,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,1},
/* row 41 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 42 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 43 */ {1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 44 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 45 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 46 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
/* row 47 */ {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
};

// ============================================================================
// --- Player State Structure ---
// ============================================================================
typedef struct {
    int32_t x, y;           
    int32_t dirX, dirY;     
    int32_t planeX, planeY;  
} RayPlayer_t;

RayPlayer_t Hero = {
    FLOAT_TO_FP(24.0f), FLOAT_TO_FP(24.0f), 
    FLOAT_TO_FP(-1.0f), FLOAT_TO_FP(0.0f), 
    FLOAT_TO_FP(0.0f), FLOAT_TO_FP(0.66f)
};

typedef enum {
    STATE_START_SCREEN = 0,
    STATE_PLAYING = 1,
    STATE_GAME_OVER = 2,
    STATE_ROUND_SPLASH = 3
} GameState_t;

GameState_t currentGameState = STATE_START_SCREEN;
uint32_t stateChanged = 1;        
uint32_t survivalScoreRound = 0;  
uint32_t playerHP    = 3;
uint32_t playerMaxHP = 3;  /* grows if full-heal reward is used at higher HP */
uint32_t currentRound = 0;        /* round counter, starts at 1 on first wave */
uint32_t splashTimer = 0;         /* counts down frames during round splash */
#define SPLASH_FRAMES 90          /* 3 seconds at 30 fps */
#define MAX_DRONES    8           /* hard cap so we don't blow the SpriteList array */

uint32_t isShooting = 0;          
uint32_t shootAnimateTimer = 0;

// ============================================================================
// --- Ability System ---
// ============================================================================

// --- Vampiric Glass Cannon (slide potentiometer) ---
// pot ~4095 = slider left  = idle, no effect, bar recharges passively
// pot ~0    = slider right = both effects active:
//   GLASS CANNON: deal 3x damage, take 2x damage
//   VAMPIRIC:     every kill restores 1 HP
// Bar drains continuously while active — faster the further right you push.
// Bar hits 0 = both effects cut off until recharged. Pickups refill bar.
#define VGC_BAR_MAX     300           /* max energy units                        */
#define VGC_BAR_PICKUP  120           /* units gained from one energy pickup     */
#define VGC_THRESHOLD   1000          /* pot must be BELOW this to activate (slider left) */
/* Drain scales with how far right the slider is pushed:
   drain = 1 + (3000 - pot) * 4 / 3000  → range 1 (barely active) to 5 (max) */
static int32_t  vgcBar    = VGC_BAR_MAX;
static uint32_t vgcActive = 0;        /* 1 when slider right AND bar > 0        */

// --- EMP Blast (B3) ---
#define EMP_FREEZE_FRAMES   90
#define EMP_COOLDOWN_FRAMES 180
static uint32_t empFreezeTimer = 0;
static uint32_t empCooldown    = 0;

// --- Melee (B2) ---
static uint32_t meleeCooldown = 0;
#define MELEE_COOLDOWN_FRAMES 30
#define MELEE_RANGE_FP        FLOAT_TO_FP(1.5f)
#define NEON_ORANGE           0xFD00  /* orange for melee HUD */

// --- Reward FIFO (B4 activates front reward) ---
// Every 5 kills a random reward is pushed to the back of the queue (max 3).
typedef enum {
    REWARD_NONE       = 0,
    REWARD_SCAN       = 1,  /* reveals all drone positions for 3 seconds        */
    REWARD_NUKE       = 2,  /* instantly kills all drones                       */
    REWARD_FULLHEAL   = 3,  /* restores HP to max                               */
    REWARD_OCSURGE    = 4,  /* instantly fills VGC (vampiric/glass cannon) bar  */
    REWARD_EMP        = 5,  /* free EMP regardless of cooldown                  */
    REWARD_SPEEDDEMON = 6,  /* player moves 3x for 5 seconds                    */
    REWARD_CONFUSION  = 7,  /* drones wander randomly for 5 seconds             */
    REWARD_DOUBLEDMG  = 8   /* all attacks one-shot for 8 seconds               */
} Reward_t;

#define REWARD_QUEUE_SIZE  3
#define KILLS_PER_REWARD   5
#define SCAN_FRAMES        90   /* 3 seconds                                    */
#define SPEED_DEMON_FRAMES 150  /* 5 seconds                                    */
#define CONFUSION_FRAMES   150  /* 5 seconds                                    */
#define DOUBLE_DMG_FRAMES  240  /* 8 seconds                                    */
#define SPEED_DEMON_MOVE   19660 /* 3x player speed                             */

static Reward_t rewardQueue[REWARD_QUEUE_SIZE];
static uint32_t rewardHead   = 0; /* index of front reward                      */
static uint32_t rewardCount  = 0; /* how many rewards currently queued          */
static uint32_t nextKillMilestone = KILLS_PER_REWARD;
static uint32_t rewardNotifyTimer = 0; /* frames to show "REWARD!" banner        */
static char     rewardName[12]    = "";

/* Active reward effect timers */
static uint32_t scanTimer       = 0;
static uint32_t speedDemonTimer = 0;
static uint32_t confusionTimer  = 0;
static uint32_t doubleDmgTimer  = 0;

// Energy pickups
#define MAX_ENERGY_PICKUPS 3
typedef struct { int32_t x, y; uint32_t active; } EnergyPickup_t;
static EnergyPickup_t EnergyList[MAX_ENERGY_PICKUPS];

// Screen flash feedback
static uint32_t flashTimer = 0;
static uint16_t flashColor = 0;

typedef enum {
    TYPE_ATTACK_DRONE,
    TYPE_PASSIVE_DRONE
} SpriteType_t;

typedef struct {
    int32_t x;          
    int32_t y;          
    int32_t hp;         
    uint32_t active;    
    SpriteType_t type;  
    int32_t distance;   
} Entity_t; 

#define NUM_SPRITES MAX_DRONES
Entity_t SpriteList[NUM_SPRITES]; 

static uint32_t randSeed = 54321; 
static uint32_t droneAttackCooldown = 0; 

// ============================================================================
// --- SYSTEM LOGIC IMPLEMENTATIONS ---
// ============================================================================

void Update_Health_LEDs(void) {
    if (playerHP == 3) {
        LED_On(0x00038000); 
    } 
    else if (playerHP == 2) {
        LED_Off(0x00008000); 
        LED_On(0x00030000);  
    } 
    else if (playerHP == 1) {
        LED_Off(0x00018000); 
        LED_On(0x00020000);  
    } 
    else {
        LED_Off(0x00038000); 
        currentGameState = STATE_GAME_OVER;
        stateChanged = 1;
    }
}

uint32_t RandomRange(uint32_t min, uint32_t max) {
    randSeed = (randSeed * 1664525 + 1013904223); 
    return (randSeed % (max - min + 1)) + min;
}

int32_t FixedAbs(int32_t n) {
    return (n < 0) ? -n : n;
}

void SpawnEnergyPickups(void) {
    for (int i = 0; i < MAX_ENERGY_PICKUPS; i++) {
        uint32_t valid = 0;
        uint32_t rx = 0, ry = 0;
        while (!valid) {
            rx = RandomRange(2, MAP_W - 3);
            ry = RandomRange(2, MAP_H - 3);
            if (worldMap[ry][rx] != 0) continue;
            if (FixedAbs(INT_TO_FP(rx) - Hero.x) <= FLOAT_TO_FP(2.0f)) continue;
            valid = 1;
        }
        EnergyList[i].x      = INT_TO_FP(rx) + FLOAT_TO_FP(0.5f);
        EnergyList[i].y      = INT_TO_FP(ry) + FLOAT_TO_FP(0.5f);
        EnergyList[i].active = 1;
    }
}

void ResetSurvivalRound(void) {
    currentRound++;

    /* Drone count grows each round: round 1 = 2, round 2 = 3, ..., capped at MAX_DRONES */
    uint32_t dronesThisRound = 1 + currentRound;
    if (dronesThisRound > MAX_DRONES) dronesThisRound = MAX_DRONES;

    /* Deactivate all slots first */
    for (int i = 0; i < NUM_SPRITES; i++) SpriteList[i].active = 0;

    for (uint32_t i = 0; i < dronesThisRound; i++) {
        uint32_t validSpawn = 0;
        uint32_t rx = 0, ry = 0;

        while (!validSpawn) {
            rx = RandomRange(2, MAP_W - 3);
            ry = RandomRange(2, MAP_H - 3);
            if (worldMap[ry][rx] != 0) continue;
            if (FixedAbs(INT_TO_FP(rx) - Hero.x) <= FLOAT_TO_FP(3.0f)) continue;
            uint32_t tooClose = 0;
            for (uint32_t j = 0; j < i; j++) {
                if (FixedAbs(INT_TO_FP(rx) - SpriteList[j].x) < FLOAT_TO_FP(2.0f) &&
                    FixedAbs(INT_TO_FP(ry) - SpriteList[j].y) < FLOAT_TO_FP(2.0f)) {
                    tooClose = 1; break;
                }
            }
            if (!tooClose) validSpawn = 1;
        }

        SpriteList[i].x      = INT_TO_FP(rx) + FLOAT_TO_FP(0.5f);
        SpriteList[i].y      = INT_TO_FP(ry) + FLOAT_TO_FP(0.5f);
        SpriteList[i].active = 1;
        SpriteList[i].type   = TYPE_ATTACK_DRONE;
        SpriteList[i].hp     = 1 + (int32_t)i;  /* drone 0=1hp, 1=2hp, etc. */
    }
    droneAttackCooldown = 30;
    /* Reset per-round ability state */
    empFreezeTimer  = 0;
    empCooldown     = 0;
    meleeCooldown   = 0;
    /* Reset active reward effect timers — rewards carry between rounds */
    /* but active effects do not, so clear them */
    speedDemonTimer = 0;
    confusionTimer  = 0;
    doubleDmgTimer  = 0;
    scanTimer       = 0;
    SpawnEnergyPickups();

    /* Show the round splash screen before gameplay resumes */
    currentGameState = STATE_ROUND_SPLASH;
    splashTimer = SPLASH_FRAMES;
    stateChanged = 1;
}

void BufferWeapon(int16_t x, int16_t y, const uint16_t *image, int16_t w, int16_t h) {
    int localWidth = currentChunkXEnd - currentChunkXStart; 
    
    for (int16_t j = 0; j < h; j++) {
        int16_t screenY = (y - h) + j; 
        if (screenY < 0 || screenY >= SCR_H) continue; 

        for (int16_t i = 0; i < w; i++) {
            int16_t screenX = x + i;
            if (screenX >= currentChunkXStart && screenX < currentChunkXEnd) {
                uint16_t color = image[j * w + i]; 
                if (color != BLACK) { 
                    int localX = screenX - currentChunkXStart; 
                    screenBuf[screenY * localWidth + localX] = color;
                }
            }
        }
    }
}

void BufferChar(int16_t x, int16_t y, char c, uint16_t color) {
    if (c < 32 || c > 90) return; 
    int localWidth = currentChunkXEnd - currentChunkXStart;
    
    const uint8_t *charPtr = &LocalFontTable[(c - 32) * 5]; 
    for (int8_t col = 0; col < 5; col++) {
        int16_t screenX = x + col;
        if (screenX >= currentChunkXStart && screenX < currentChunkXEnd) {
            int localX = screenX - currentChunkXStart;
            uint8_t line = charPtr[col]; 
            for (int8_t row = 0; row < 8; row++) {
                if (line & (1 << (7 - row))) {
                    int16_t screenY = y + row;
                    if (screenY >= 0 && screenY < SCR_H) {
                        screenBuf[screenY * localWidth + localX] = color;
                    }
                }
            }
        }
    }
}

void BufferString(int16_t x, int16_t y, char *str, uint16_t color) {
    while (*str) {
        BufferChar(x, y, *str, color);
        x += 6; 
        str++;
    }
}

void CheckWeaponHitscan(void) {
    for(int i = 0; i < NUM_SPRITES; i++) {
        if(!SpriteList[i].active) continue;
        
        int32_t spriteX = SpriteList[i].x - Hero.x;
        int32_t spriteY = SpriteList[i].y - Hero.y;
        
        int32_t invDet = FP_DIV(INT_TO_FP(1), (FP_MUL(Hero.planeX, Hero.dirY) - FP_MUL(Hero.dirX, Hero.planeY)));
        int32_t transformX = FP_MUL(invDet, (FP_MUL(Hero.dirY, spriteX) - FP_MUL(Hero.dirX, spriteY)));
        int32_t transformY = FP_MUL(invDet, (-FP_MUL(Hero.planeY, spriteX) + FP_MUL(Hero.planeX, spriteY))); 
        
        if(transformY <= FLOAT_TO_FP(0.1f)) continue; 
        
        int spriteScreenX = FP_TO_INT(FP_MUL(INT_TO_FP(SCR_W / 2), FP_DIV(transformX, transformY) + INT_TO_FP(1)));
        
        if(spriteScreenX >= 44 && spriteScreenX <= 84) {
            /* Glass cannon: 3x damage when VGC active. Double dmg reward: one-shot. */
            int32_t dmg = (doubleDmgTimer > 0) ? 99 : (vgcActive ? 3 : 1);
            SpriteList[i].hp -= dmg;
            if(SpriteList[i].hp <= 0) {
                SpriteList[i].active = 0;
                survivalScoreRound++;
                /* Vampiric: restore 1 HP on kill when VGC active */
                if (vgcActive && playerHP < playerMaxHP) {
                    playerHP++;
                    Update_Health_LEDs();
                }
            }
            break;
        }
    }
}

void PLL_Init(void){ 
    Clock_Init80MHz(0); 
}

uint32_t input, slideButton;
uint32_t flag;
volatile uint32_t rawX, rawY, pot;

void TIMG12_IRQHandler(void){
  if((TIMG12->CPU_INT.IIDX) == 1){
    /* Read all three ADC1 channels in one shot:
       d1 = MEMRES[1] = channel 4 = joystick X
       d2 = MEMRES[2] = channel 5 = slide pot
       d3 = MEMRES[3] = channel 6 = joystick Y */
    uint32_t jx, sp, jy;
    ADC_InTriple(ADC1, &jx, &sp, &jy);
    rawX = jx;
    pot  = sp;
    rawY = jy;
    input      = Switch_In();
    slideButton = JoyStick_InButton();
    randSeed   += rawX + rawY;
  }
  flag = 1;
}

const char Hello_English[] = "Survival Mode";
const Japan_t Hello_Japanese[] = { J_KO, J_N, J_NI, J_CHI, J_WA, J_NULL };
const char Start_English[] = "Survive!";
const Japan_t Start_Japanese[] = { J_GE, J_CHO, J_MU, J_SU, J_TA, J_CHO, J_KO, J_NULL };
typedef enum { English, Japanese } Language_t;
typedef enum { HELLO, Start } Phrase_t; 

const void* Phrases[2][2] = {
  { (void*)Hello_English,  (void*)Hello_Japanese },
  { (void*)Start_English,  (void*)Start_Japanese}
};

Language_t myLanguage = English;

void DisplayPhrase(Phrase_t message, uint32_t x, uint32_t y) {
    if(myLanguage == English) {
        ST7735_SetCursor(x/6, y/10); 
        ST7735_OutString((char*)Phrases[message][English]);
    } 
    else {
        Japan_SetCursor(x, y);
        Japan_OutString((Japan_t*)Phrases[message][Japanese]);
    }
}

void RenderHalf(int xStart, int xEnd) {
    int localWidth = xEnd - xStart;
    for (int x = xStart; x < xEnd; x++) {
        int localX = x - xStart; 
        
        int32_t cameraX = FP_DIV(INT_TO_FP(2 * x), INT_TO_FP(SCR_W)) - INT_TO_FP(1);
        int32_t rayDirX = Hero.dirX + FP_MUL(Hero.planeX, cameraX);
        int32_t rayDirY = Hero.dirY + FP_MUL(Hero.planeY, cameraX);

        int mapX = FP_TO_INT(Hero.x);
        int mapY = FP_TO_INT(Hero.y);

        int32_t deltaDistX = (rayDirX == 0) ? 0x7FFFFFFF : FixedAbs(FP_DIV(INT_TO_FP(1), rayDirX));
        int32_t deltaDistY = (rayDirY == 0) ? 0x7FFFFFFF : FixedAbs(FP_DIV(INT_TO_FP(1), rayDirY));

        int32_t sideDistX, sideDistY; 
        int stepX = (rayDirX < 0) ? -1 : 1;
        int stepY = (rayDirY < 0) ? -1 : 1;             

        if (rayDirX < 0) {
            sideDistX = FP_MUL((Hero.x - INT_TO_FP(mapX)), deltaDistX);
        } else {
            sideDistX = FP_MUL((INT_TO_FP(mapX + 1) - Hero.x), deltaDistX);
        }
        
        if (rayDirY < 0) {
            sideDistY = FP_MUL((Hero.y - INT_TO_FP(mapY)), deltaDistY);
        } else {
            sideDistY = FP_MUL((INT_TO_FP(mapY + 1) - Hero.y), deltaDistY); 
        }

        int side;    
        while (1) {
            if (sideDistX < sideDistY) {
                sideDistX += deltaDistX;
                mapX += stepX;
                side = 0; 
            } else {
                sideDistY += deltaDistY;
                mapY += stepY;
                side = 1; 
            }
            if(mapX >= MAP_W || mapY >= MAP_H || mapX < 0 || mapY < 0) break; 
            if (worldMap[mapY][mapX] > 0) break; 
        }

        int32_t perpWallDist = (side == 0) ? (sideDistX - deltaDistX) : (sideDistY - deltaDistY);
        if(perpWallDist <= 0) perpWallDist = 1; 
        
        ZBuffer[x] = perpWallDist; 
        
        int lineHeight = FP_TO_INT(FP_DIV(INT_TO_FP(SCR_H), perpWallDist));
        
        int drawStart = -lineHeight / 2 + SCR_H / 2;
        if (drawStart < 0) drawStart = 0; 
        int drawEnd = lineHeight / 2 + SCR_H / 2;
        if (drawEnd >= SCR_H) drawEnd = SCR_H - 1; 

        uint16_t color = (side == 1) ? NEON_PINK : NEON_CYAN;

        for (int y = 0; y < SCR_H; y++) {
            if (y < drawStart) {
                screenBuf[y * localWidth + localX] = BLACK;      
            } else if (y <= drawEnd) {
                screenBuf[y * localWidth + localX] = color;      
            } else {
                screenBuf[y * localWidth + localX] = 0x2104;     
            }
        }
    }
}

void SortSprites(Entity_t* sprites, int count) {
    for (int i = 0; i < count - 1; i++) {
        for (int j = i + 1; j < count; j++) {
            if (sprites[i].distance < sprites[j].distance) {
                Entity_t temp = sprites[i];
                sprites[i] = sprites[j];
                sprites[j] = temp;
            }
        }
    }
}

// ============================================================================
// --- BUG FIXED: ALL SPRITE INVISIBILITY & NOISE ARTIFACT ENGINE ---
// ============================================================================
void RenderAllSpritesToBuffer(void) {
    for (int i = 0; i < NUM_SPRITES; i++) {
        if (!SpriteList[i].active) {
            SpriteList[i].distance = -1;
            continue;
        }
        int32_t dx = SpriteList[i].x - Hero.x;
        int32_t dy = SpriteList[i].y - Hero.y;
        SpriteList[i].distance = FP_MUL(dx, dx) + FP_MUL(dy, dy); 
    }

    SortSprites(SpriteList, NUM_SPRITES);

    int localWidth = currentChunkXEnd - currentChunkXStart; 

    for (int i = 0; i < NUM_SPRITES; i++) {
        if (!SpriteList[i].active || SpriteList[i].distance < 0) continue;

        const uint16_t* texture = attackDrone; 
        int texW = 52; 
        int texH = 35;

        int32_t spriteX = SpriteList[i].x - Hero.x;
        int32_t spriteY = SpriteList[i].y - Hero.y;

        int32_t invDet = FP_DIV(INT_TO_FP(1), (FP_MUL(Hero.planeX, Hero.dirY) - FP_MUL(Hero.dirX, Hero.planeY)));
        int32_t transformX = FP_MUL(invDet, (FP_MUL(Hero.dirY, spriteX) - FP_MUL(Hero.dirX, spriteY)));
        int32_t transformY = FP_MUL(invDet, (-FP_MUL(Hero.planeY, spriteX) + FP_MUL(Hero.planeX, spriteY))); 

        if (transformY <= FLOAT_TO_FP(0.1f)) continue; 

        int spriteScreenX = FP_TO_INT(FP_MUL(INT_TO_FP(SCR_W / 2), FP_DIV(transformX, transformY) + INT_TO_FP(1)));
        int spriteHeight = FixedAbs(FP_TO_INT(FP_DIV(INT_TO_FP(SCR_H), transformY)));
        int spriteWidth  = spriteHeight;

        if (spriteHeight <= 0 || spriteWidth <= 0) continue;

        int drawStartY = -spriteHeight / 2 + SCR_H / 2;
        int drawEndY = spriteHeight / 2 + SCR_H / 2;
        if (drawStartY < 0) drawStartY = 0;
        if (drawEndY >= SCR_H) drawEndY = SCR_H - 1;

        int unclippedStartX = -spriteWidth / 2 + spriteScreenX;
        int drawStartX = unclippedStartX;
        int drawEndX = spriteWidth / 2 + spriteScreenX;
        
        if (drawStartX < 0) drawStartX = 0;
        if (drawEndX >= SCR_W) drawEndX = SCR_W - 1;

        for (int stripe = drawStartX; stripe < drawEndX; stripe++) {
            if (stripe >= currentChunkXStart && stripe < currentChunkXEnd) {
                if (transformY < ZBuffer[stripe]) {
                    int localX = stripe - currentChunkXStart;
                    
                    if (localX >= 0 && localX < localWidth) {
                        int texX = ((stripe - unclippedStartX) * texW) / spriteWidth;
                        
                        if (texX >= 0 && texX < texW) {
                            for (int y = drawStartY; y < drawEndY; y++) {
                                int texY = ((y - (-spriteHeight / 2 + SCR_H / 2)) * texH) / spriteHeight;
                                
                                if (texY >= 0 && texY < texH) {
                                    uint16_t color = texture[texY * texW + texX];
                                    if (color != BLACK) { 
                                        screenBuf[y * localWidth + localX] = color;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        
        // ====================================================================
        // --- FIXED: Enemy Projectile Horizontal Centering and Tracking ---
        // ====================================================================
        if(SpriteList[i].type == TYPE_ATTACK_DRONE && transformY < FLOAT_TO_FP(5.0f)) {
            int16_t projectileX = 58;
            BufferWeapon(projectileX, 85, ball1, 12, 12); 
        }
    }

    /* --- Energy pickup billboards (16x16 sprite) --- */
    for (int i = 0; i < MAX_ENERGY_PICKUPS; i++) {
        if (!EnergyList[i].active) continue;

        int32_t eX = EnergyList[i].x - Hero.x;
        int32_t eY = EnergyList[i].y - Hero.y;
        int32_t invDet = FP_DIV(INT_TO_FP(1), (FP_MUL(Hero.planeX, Hero.dirY) - FP_MUL(Hero.dirX, Hero.planeY)));
        int32_t etX = FP_MUL(invDet, (FP_MUL(Hero.dirY, eX) - FP_MUL(Hero.dirX, eY)));
        int32_t etY = FP_MUL(invDet, (-FP_MUL(Hero.planeY, eX) + FP_MUL(Hero.planeX, eY)));
        if (etY <= FLOAT_TO_FP(0.1f)) continue;

        int eScrX   = FP_TO_INT(FP_MUL(INT_TO_FP(SCR_W / 2), FP_DIV(etX, etY) + INT_TO_FP(1)));
        int eHeight = FixedAbs(FP_TO_INT(FP_DIV(INT_TO_FP(SCR_H), etY)));
        if (eHeight <= 0) continue;

        int eDSY = -eHeight / 2 + SCR_H / 2; if (eDSY < 0) eDSY = 0;
        int eDEY = eHeight / 2 + SCR_H / 2;  if (eDEY >= SCR_H) eDEY = SCR_H - 1;
        int eUncX = -eHeight / 2 + eScrX;
        int eDSX = eUncX;                    if (eDSX < 0) eDSX = 0;
        int eDEX = eHeight / 2 + eScrX;     if (eDEX >= SCR_W) eDEX = SCR_W - 1;

        for (int stripe = eDSX; stripe < eDEX; stripe++) {
            if (stripe < currentChunkXStart || stripe >= currentChunkXEnd) continue;
            if (etY >= ZBuffer[stripe]) continue;
            int localX = stripe - currentChunkXStart;
            int texX = ((stripe - eUncX) * 16) / eHeight;
            if (texX < 0 || texX >= 16) continue;
            for (int y = eDSY; y < eDEY; y++) {
                int texY = ((y - (-eHeight / 2 + SCR_H / 2)) * 16) / eHeight;
                if (texY < 0 || texY >= 16) continue;
                uint16_t col = energy[texY * 16 + texX];
                if (col != BLACK) screenBuf[y * localWidth + localX] = col;
            }
        }
    }
}

// ============================================================================
// --- Line-of-Sight Check (DDA grid walk) ---
// Returns 1 if there is a clear path between (ax,ay) and (bx,by) in map space.
// Both coords are 16.16 fixed-point world positions.
// ============================================================================
static uint32_t HasLineOfSight(int32_t ax, int32_t ay, int32_t bx, int32_t by) {
    int32_t dx = bx - ax;
    int32_t dy = by - ay;

    /* Use ~32 steps regardless of distance — enough for a 48x48 map */
    int steps = 32;
    int32_t stepX = dx / steps;
    int32_t stepY = dy / steps;

    int32_t cx = ax;
    int32_t cy = ay;
    for (int s = 0; s < steps; s++) {
        cx += stepX;
        cy += stepY;
        int gx = FP_TO_INT(cx);
        int gy = FP_TO_INT(cy);
        if (gx < 0 || gx >= MAP_W || gy < 0 || gy >= MAP_H) return 0;
        if (worldMap[gy][gx] != 0) return 0;  /* wall in the way */
    }
    return 1;
}

// ============================================================================
// --- MAIN PROGRAM ENTRY POINT ---
// ============================================================================
int main(void){ 
  __disable_irq(); 
  PLL_Init(); 
  LaunchPad_Init();
  ST7735_InitPrintf(INITR_REDTAB); 
  ST7735_FillScreen(ST7735_BLACK);
  /* Initialize ADC1 for all three channels at once:
     channel 4 = joystick X (PB17)
     channel 5 = slide pot  (PB18)
     channel 6 = joystick Y (PB19)
     This avoids ADCinit and JoyStick_Init fighting over ADC1 config */
  ADC_InitTriple(ADC1, 4, 5, 6, ADCVREF_VDDA);
  Switch_Init(); 
  LED_Init();    
  Sound_Init();  
  
  LED_Off(0x00038000);   
  /* Drones are spawned when the player presses B1 on the start screen */
  
  TimerG12_IntArm(80000000/30, 2); 
  __enable_irq(); 
  
  uint32_t lastInput = 0;
  char hudString[16];
  
  while(1){
     if(flag == 1){ 
        flag = 0; 
        
        switch(currentGameState){
        
        case STATE_START_SCREEN:
            if(stateChanged){
                ST7735_FillScreen(BLACK);
                ST7735_SetTextColor(NEON_CYAN);
                ST7735_SetCursor(0, 0);  ST7735_OutString("====================="); 
                ST7735_SetCursor(0, 15); ST7735_OutString("====================="); 
                ST7735_SetTextColor(NEON_PINK);
                ST7735_SetCursor(1, 1);  ST7735_OutString("-------------------");
                ST7735_SetCursor(1, 14); ST7735_OutString("-------------------");
                ST7735_SetTextColor(NEON_CYAN);
                ST7735_SetCursor(4, 3);  ST7735_OutString("NEON");
                ST7735_SetTextColor(NEON_PINK);
                ST7735_SetCursor(3, 5);  ST7735_OutString("GENESIS");
                
                ST7735_DrawTransparentmap(64,80, Demon, 52,52);
                ST7735_SetTextColor(ST7735_WHITE);
                if(myLanguage == English) {
                    ST7735_SetCursor(2, 10); ST7735_OutString("B1: "); 
                    ST7735_OutString((char*)Phrases[Start][English]);
                    ST7735_SetTextColor(NEON_YELLOW); 
                    ST7735_SetCursor(2, 12); 
                    ST7735_OutString("B2 Lang: EN");
                } 
                else {
                    ST7735_SetCursor(2, 10); ST7735_OutString("B1: "); 
                    DisplayPhrase(Start, 30,100);
                    ST7735_SetTextColor(NEON_YELLOW); 
                    ST7735_SetCursor(2, 12); 
                    ST7735_OutString("B2 Lang: JP");
                }
                stateChanged = 0;
            }
            if(input != lastInput) { 
                if(input == 2) { 
                    myLanguage = (myLanguage == English) ? Japanese : English;
                    stateChanged = 1;
                }
                if(input == 1){ 
                    Sound_shoot();     
                    stateChanged = 1;   
                    Hero.x = FLOAT_TO_FP(24.0f); 
                    Hero.y = FLOAT_TO_FP(24.0f);
                    survivalScoreRound = 0;
                    currentRound = 0;
                    playerHP = 3;
                    vgcBar         = VGC_BAR_MAX;
                    vgcActive      = 0;
                    empFreezeTimer = 0;
                    empCooldown    = 0;
                    meleeCooldown  = 0;
                    flashTimer     = 0;
                    /* Reset reward FIFO and kill milestone */
                    rewardHead          = 0;
                    rewardCount         = 0;
                    rewardNotifyTimer   = 0;
                    nextKillMilestone   = KILLS_PER_REWARD;
                    scanTimer           = 0;
                    speedDemonTimer     = 0;
                    confusionTimer      = 0;
                    doubleDmgTimer      = 0;
                    playerMaxHP         = 3;
                    currentGameState = STATE_PLAYING;
                    Update_Health_LEDs();
                    ResetSurvivalRound();
                }
                lastInput = input; 
            }
            break;

         case STATE_PLAYING: 
            if(stateChanged){
                ST7735_FillScreen(BLACK); 
                stateChanged = 0;
            }

            /* ================================================================
               VAMPIRIC GLASS CANNON (slide potentiometer)
               pot ~4095 = slider right = idle, no effect
               pot ~0    = slider left  = glass cannon + vampiric active
               Bar ONLY recharges from energy pickups — no passive regen.
               ================================================================ */
            {
                uint32_t currentPot = pot;
                if (currentPot > 4095) currentPot = 4095;

                /* Active when pot is LOW (slider pushed left toward 0) */
                if (currentPot < VGC_THRESHOLD && vgcBar > 0) {
                    vgcActive = 1;
                    /* drain = 1 (just past threshold) up to 5 (fully left at 0) */
                    uint32_t pushed = VGC_THRESHOLD - currentPot; /* 0..VGC_THRESHOLD */
                    uint32_t drain  = 1 + (pushed * 4 / VGC_THRESHOLD);
                    if (drain > 5) drain = 5;
                    vgcBar -= (int32_t)drain;
                    if (vgcBar < 0) { vgcBar = 0; vgcActive = 0; }
                } else {
                    /* Idle — no passive regen, only energy pickups refill */
                    vgcActive = 0;
                }
            }

            /* ================================================================
               COOLDOWN TICKS — decrement all per-frame timers
               ================================================================ */
            if (empCooldown    > 0) empCooldown--;
            if (empFreezeTimer > 0) empFreezeTimer--;
            if (meleeCooldown  > 0) meleeCooldown--;

            /* ================================================================
               ACTIVE REWARD EFFECT TIMERS
               ================================================================ */
            if (scanTimer       > 0) scanTimer--;
            if (speedDemonTimer > 0) speedDemonTimer--;
            if (confusionTimer  > 0) confusionTimer--;
            if (doubleDmgTimer  > 0) doubleDmgTimer--;
            if (rewardNotifyTimer > 0) rewardNotifyTimer--;

            /* ================================================================
               PLAYER MOVEMENT
               Speed demon reward triples move speed, otherwise normal.
               ================================================================ */
            {
                int32_t activeMoveSpeed = (speedDemonTimer > 0) ? SPEED_DEMON_MOVE : MOVE_SPEED;
                int32_t activeCos       = (speedDemonTimer > 0) ? 64963 : COS_VAL;
                int32_t activeSin       = (speedDemonTimer > 0) ? 9806  : SIN_VAL;

                /* Rotate right */
                if(rawX > 3000) {
                    int32_t oldDirX = Hero.dirX;
                    Hero.dirX   = FP_MUL(Hero.dirX,  activeCos) - FP_MUL(Hero.dirY,  activeSin);
                    Hero.dirY   = FP_MUL(oldDirX,    activeSin) + FP_MUL(Hero.dirY,  activeCos);
                    int32_t oldPlaneX = Hero.planeX;
                    Hero.planeX = FP_MUL(Hero.planeX, activeCos) - FP_MUL(Hero.planeY, activeSin);
                    Hero.planeY = FP_MUL(oldPlaneX,   activeSin) + FP_MUL(Hero.planeY, activeCos);
                }
                /* Rotate left */
                if(rawX < 1000) {
                    int32_t oldDirX = Hero.dirX;
                    Hero.dirX   = FP_MUL(Hero.dirX,  activeCos) + FP_MUL(Hero.dirY,  activeSin);
                    Hero.dirY   = -FP_MUL(oldDirX,   activeSin) + FP_MUL(Hero.dirY,  activeCos);
                    int32_t oldPlaneX = Hero.planeX;
                    Hero.planeX = FP_MUL(Hero.planeX, activeCos) + FP_MUL(Hero.planeY, activeSin);
                    Hero.planeY = -FP_MUL(oldPlaneX,  activeSin) + FP_MUL(Hero.planeY, activeCos);
                }

                /* Move forward/backward with wall collision */
                int currentP_X = FP_TO_INT(Hero.x);
                int currentP_Y = FP_TO_INT(Hero.y);
                int32_t nextX = Hero.x, nextY = Hero.y;
                int moved = 0;

                if(rawY > 3000) {
                    nextX = Hero.x + FP_MUL(Hero.dirX, activeMoveSpeed);
                    nextY = Hero.y + FP_MUL(Hero.dirY, activeMoveSpeed);
                    moved = 1;
                }
                if(rawY < 1000) {
                    nextX = Hero.x - FP_MUL(Hero.dirX, activeMoveSpeed);
                    nextY = Hero.y - FP_MUL(Hero.dirY, activeMoveSpeed);
                    moved = 1;
                }
                if(moved) {
                    int checkX = FP_TO_INT(nextX);
                    int checkY = FP_TO_INT(nextY);
                    if(checkX >= 0 && checkX < MAP_W && checkY >= 0 && checkY < MAP_H) {
                        if(worldMap[currentP_Y][checkX] == 0) Hero.x = nextX;
                        if(worldMap[checkY][currentP_X] == 0) Hero.y = nextY;
                    }
                }
            }

            /* ================================================================
               DRONE AI MOVEMENT
               - Frozen if EMP is active
               - Confused if confusion reward is active (move randomly)
               - Overclocked if OC is active (move faster)
               ================================================================ */
            for (int i = 0; i < NUM_SPRITES; i++) {
                if (!SpriteList[i].active) continue;

                /* EMP freeze: skip all drone movement */
                if (empFreezeTimer > 0) continue;

                int32_t stepEnemyX = 0, stepEnemyY = 0;
                int32_t droneSpd = DRONE_SPEED;

                if (confusionTimer > 0) {
                    /* Confusion reward: drones move in a random direction each frame */
                    uint32_t rdir = RandomRange(0, 3);
                    if (rdir == 0) stepEnemyX =  droneSpd;
                    if (rdir == 1) stepEnemyX = -droneSpd;
                    if (rdir == 2) stepEnemyY =  droneSpd;
                    if (rdir == 3) stepEnemyY = -droneSpd;
                } else {
                    /* Normal: chase the player */
                    int32_t vX = Hero.x - SpriteList[i].x;
                    int32_t vY = Hero.y - SpriteList[i].y;
                    if (vX > FLOAT_TO_FP(0.1f))  stepEnemyX =  droneSpd;
                    if (vX < FLOAT_TO_FP(-0.1f)) stepEnemyX = -droneSpd;
                    if (vY > FLOAT_TO_FP(0.1f))  stepEnemyY =  droneSpd;
                    if (vY < FLOAT_TO_FP(-0.1f)) stepEnemyY = -droneSpd;
                }

                int32_t enNextX = SpriteList[i].x + stepEnemyX;
                int32_t enNextY = SpriteList[i].y + stepEnemyY;
                int curGridY    = FP_TO_INT(SpriteList[i].y);
                int nextGridX   = FP_TO_INT(enNextX);
                int nextGridY   = FP_TO_INT(enNextY);

                if (nextGridX >= 0 && nextGridX < MAP_W)
                    if (worldMap[curGridY][nextGridX] == 0) SpriteList[i].x = enNextX;
                if (nextGridY >= 0 && nextGridY < MAP_H)
                    if (worldMap[nextGridY][FP_TO_INT(SpriteList[i].x)] == 0) SpriteList[i].y = enNextY;
            }

            /* ================================================================
               DRONE DAMAGE CHECK
               Only triggers when EMP is not active and drone is close + in LOS.
               ================================================================ */
            if (droneAttackCooldown > 0) {
                droneAttackCooldown--;
            } else if (empFreezeTimer == 0) {
                for (int i = 0; i < NUM_SPRITES; i++) {
                    if (!SpriteList[i].active) continue;
                    int32_t spriteX    = SpriteList[i].x - Hero.x;
                    int32_t spriteY    = SpriteList[i].y - Hero.y;
                    int32_t invDet     = FP_DIV(INT_TO_FP(1), (FP_MUL(Hero.planeX, Hero.dirY) - FP_MUL(Hero.dirX, Hero.planeY)));
                    int32_t transformY = FP_MUL(invDet, (-FP_MUL(Hero.planeY, spriteX) + FP_MUL(Hero.planeX, spriteY)));
                    if (SpriteList[i].type == TYPE_ATTACK_DRONE
                        && transformY > 0 && transformY < FLOAT_TO_FP(3.0f)
                        && HasLineOfSight(SpriteList[i].x, SpriteList[i].y, Hero.x, Hero.y)) {
                        /* Glass cannon: take 2x damage when VGC active */
                        int32_t dmgTaken = vgcActive ? 2 : 1;
                        playerHP -= dmgTaken;
                        if (playerHP < 0) playerHP = 0;
                        flashColor = NEON_PINK;
                        flashTimer = 6;
                        Update_Health_LEDs();
                        droneAttackCooldown = 45;
                        break;
                    }
                }
            }

            /* ================================================================
               BUTTON INPUTS
               B1 = Shoot (rapid fire when OC active)
               B2 = Melee stomp (kills drones within 1.5 units)
               B3 = EMP blast (freezes all drones 3 seconds)
               B4 = Activate front reward from FIFO queue
               ================================================================ */

            /* B1: Shoot — single shot, glass cannon dmg applied in hitscan */
            if((input == 1) && (lastInput != 1) && (!isShooting)) {
                Sound_shoot();
                isShooting        = 1;
                shootAnimateTimer = 4;
                CheckWeaponHitscan();
            }

            /* B2: Melee stomp — orange flash, kills drones within MELEE_RANGE_FP */
            if((input == 2) && (lastInput != 2) && (meleeCooldown == 0)) {
                meleeCooldown = MELEE_COOLDOWN_FRAMES;
                flashColor    = NEON_ORANGE; /* orange flash for melee */
                flashTimer    = 4;
                for (int i = 0; i < NUM_SPRITES; i++) {
                    if (!SpriteList[i].active) continue;
                    int32_t dx     = SpriteList[i].x - Hero.x;
                    int32_t dy     = SpriteList[i].y - Hero.y;
                    int32_t distSq = FP_MUL(dx, dx) + FP_MUL(dy, dy);
                    /* Glass cannon 3x dmg, double dmg reward = one-shot */
                    int32_t dmg = (doubleDmgTimer > 0) ? 99 : (vgcActive ? 3 : 1);
                    if (distSq < FP_MUL(MELEE_RANGE_FP, MELEE_RANGE_FP)) {
                        SpriteList[i].hp -= dmg;
                        if (SpriteList[i].hp <= 0) {
                            SpriteList[i].active = 0;
                            survivalScoreRound++;
                            /* Vampiric: restore 1 HP on melee kill when VGC active */
                            if (vgcActive && playerHP < playerMaxHP) {
                                playerHP++;
                                Update_Health_LEDs();
                            }
                        }
                    }
                }
            }

            /* B3: EMP blast — freezes all drones for EMP_FREEZE_FRAMES */
            if((input == 4) && (lastInput != 4) && (empCooldown == 0)) {
                empFreezeTimer = EMP_FREEZE_FRAMES;
                empCooldown    = EMP_COOLDOWN_FRAMES;
                flashColor     = NEON_YELLOW;
                flashTimer     = 8;
                Sound_shoot();
            }

            /* B4: Activate front reward from FIFO queue */
            if((input == 8) && (lastInput != 8) && (rewardCount > 0)) {
                /* Pop the front reward */
                Reward_t r = rewardQueue[rewardHead];
                rewardHead  = (rewardHead + 1) % REWARD_QUEUE_SIZE;
                rewardCount--;

                /* Apply the reward and set notify banner */
                switch(r) {
                    case REWARD_SCAN:
                        /* Scan: reveals all drone positions as dots for 3 seconds */
                        scanTimer = SCAN_FRAMES;
                        flashColor = NEON_CYAN;
                        if(myLanguage == English) sprintf(rewardName, "SCAN!");
                        else                      sprintf(rewardName, "SCAN!");
                        break;
                    case REWARD_NUKE:
                        /* Nuke: instantly kills every active drone */
                        for(int i = 0; i < NUM_SPRITES; i++) {
                            if(SpriteList[i].active) {
                                SpriteList[i].active = 0;
                                survivalScoreRound++;
                            }
                        }
                        flashColor = NEON_YELLOW;
                        if(myLanguage == English) sprintf(rewardName, "NUKE!");
                        else                      sprintf(rewardName, "NUKE!");
                        break;
                    case REWARD_FULLHEAL:
                        /* Full heal: restore HP to current max */
                        playerHP = playerMaxHP;
                        Update_Health_LEDs();
                        flashColor = NEON_GREEN;
                        if(myLanguage == English) sprintf(rewardName, "HEALED!");
                        else                      sprintf(rewardName, "HEALED!");
                        break;
                    case REWARD_OCSURGE:
                        /* VGC surge: instantly fills the vampiric/glass cannon bar */
                        vgcBar = VGC_BAR_MAX;
                        flashColor = NEON_YELLOW;
                        sprintf(rewardName, "VGC FULL!");
                        break;
                    case REWARD_EMP:
                        /* Free EMP: ignores cooldown */
                        empFreezeTimer = EMP_FREEZE_FRAMES;
                        empCooldown    = EMP_COOLDOWN_FRAMES;
                        flashColor     = NEON_YELLOW;
                        if(myLanguage == English) sprintf(rewardName, "EMP!");
                        else                      sprintf(rewardName, "EMP!");
                        break;
                    case REWARD_SPEEDDEMON:
                        /* Speed demon: 3x player move speed for 5 seconds */
                        speedDemonTimer = SPEED_DEMON_FRAMES;
                        flashColor      = NEON_GREEN;
                        if(myLanguage == English) sprintf(rewardName, "SPEED!");
                        else                      sprintf(rewardName, "SPEED!");
                        break;
                    case REWARD_CONFUSION:
                        /* Confusion: drones move randomly for 5 seconds */
                        confusionTimer = CONFUSION_FRAMES;
                        flashColor     = NEON_PINK;
                        if(myLanguage == English) sprintf(rewardName, "CONFUSE!");
                        else                      sprintf(rewardName, "CONFUSE!");
                        break;
                    case REWARD_DOUBLEDMG:
                        /* Double damage: all attacks one-shot for 8 seconds */
                        doubleDmgTimer = DOUBLE_DMG_FRAMES;
                        flashColor     = NEON_ORANGE;
                        if(myLanguage == English) sprintf(rewardName, "2X DMG!");
                        else                      sprintf(rewardName, "2X DMG!");
                        break;
                    default: break;
                }
                flashTimer        = 8;
                rewardNotifyTimer = 60; /* show reward name for 2 seconds */
            }

            /* ================================================================
               HITSCAN DOUBLE DAMAGE HOOK
               If double damage is active, CheckWeaponHitscan already killed
               on hit (hp set to 0). We apply the dmg multiplier via drone HP.
               Actually handled below in hitscan by checking doubleDmgTimer.
               ================================================================ */

            /* ================================================================
               KILL MILESTONE CHECK
               Every KILLS_PER_REWARD kills, push a random reward to the FIFO.
               If queue is full (3 rewards), the new reward is lost.
               ================================================================ */
            if (survivalScoreRound >= nextKillMilestone) {
                nextKillMilestone += KILLS_PER_REWARD;
                if (rewardCount < REWARD_QUEUE_SIZE) {
                    /* Pick a random reward from the 8-item pool (1..8) */
                    uint32_t roll = RandomRange(1, 8);
                    uint32_t tail = (rewardHead + rewardCount) % REWARD_QUEUE_SIZE;
                    rewardQueue[tail] = (Reward_t)roll;
                    rewardCount++;
                    /* Flash and brief notify that a reward was earned */
                    flashColor        = NEON_CYAN;
                    flashTimer        = 6;
                    rewardNotifyTimer = 60;
                    sprintf(rewardName, "REWARD+");
                }
            }

            /* ================================================================
               ENERGY PICKUP COLLECTION
               Walk over a pickup to refill the OC bar.
               ================================================================ */
            for (int i = 0; i < MAX_ENERGY_PICKUPS; i++) {
                if (!EnergyList[i].active) continue;
                int32_t dx     = EnergyList[i].x - Hero.x;
                int32_t dy     = EnergyList[i].y - Hero.y;
                int32_t distSq = FP_MUL(dx, dx) + FP_MUL(dy, dy);
                if (distSq < FP_MUL(FLOAT_TO_FP(0.6f), FLOAT_TO_FP(0.6f))) {
                    EnergyList[i].active = 0;
                    /* Refill the vampiric/glass cannon bar */
                    vgcBar += VGC_BAR_PICKUP;
                    if (vgcBar > VGC_BAR_MAX) vgcBar = VGC_BAR_MAX;
                    flashColor = NEON_CYAN;
                    flashTimer = 4;
                }
            }

            lastInput = input;

            /* ================================================================
               RENDERING — two 64-wide chunks to fit in SRAM
               ================================================================ */
            for (int chunk = 0; chunk < SCR_W; chunk += 64) {
                currentChunkXStart = chunk;
                currentChunkXEnd   = chunk + 64;

                RenderHalf(currentChunkXStart, currentChunkXEnd);
                RenderAllSpritesToBuffer();

                /* Draw weapon sprite */
                if(isShooting) {
                    BufferWeapon(70, 64, plasmashot, 64, 64);
                } else {
                    BufferWeapon(70, 64, plasma, 64, 64);
                }

                /* Draw crosshair */
                for (int16_t j = 0; j < 8; j++) {
                    int16_t screenY = 75 + j;
                    for (int16_t i = 0; i < 8; i++) {
                        int16_t screenX = 60 + i;
                        if (screenX >= currentChunkXStart && screenX < currentChunkXEnd) {
                            uint16_t col = crosshair[j * 8 + i];
                            if (col != BLACK) {
                                int localX = screenX - currentChunkXStart;
                                screenBuf[screenY * 64 + localX] = col;
                            }
                        }
                    }
                }

                /* HUD: kills only, no round prefix */
                if(myLanguage == English) {
                    sprintf(hudString, "KILLS:%d", survivalScoreRound);
                } else {
                    sprintf(hudString, "K:%d", survivalScoreRound);
                }
                BufferString(4, 4, hudString, ST7735_WHITE);
                /* HUD: VGC bar same row as M and VC — starts after VC label */
                {
                    int barX = 36, barY = 152, barW = 88, barH = 6;
                    int filled = (int)((int32_t)vgcBar * barW / VGC_BAR_MAX);
                    if (filled < 0)    filled = 0;
                    if (filled > barW) filled = barW;
                    uint16_t barFill = vgcActive ? 0xF800 : NEON_GREEN;
                    for (int by = barY; by < barY + barH; by++) {
                        for (int bx = barX; bx < barX + barW; bx++) {
                            if (bx >= currentChunkXStart && bx < currentChunkXEnd) {
                                int lx      = bx - currentChunkXStart;
                                int localBx = bx - barX;
                                screenBuf[by * 64 + lx] = (localBx < filled) ? barFill : 0x2104;
                            }
                        }
                    }
                    BufferString(22, barY, "  ", ST7735_BLACK); /* clear old VC spot */
                }

                /* HUD: EMP status top-right */
                if (empFreezeTimer > 0)     BufferString(90, 4, "EMP!", NEON_YELLOW);
                else if (empCooldown > 0)   BufferString(90, 4, "EMP-", 0x7BEF);
                else                        BufferString(90, 4, "EMP+", NEON_GREEN);

                /* HUD: reward queue count */
                if (rewardCount > 0) {
                    char rStr[6];
                    sprintf(rStr, "[%d]", rewardCount);
                    BufferString(4, 14, rStr, NEON_CYAN);
                }

                /* HUD: active effect indicators */
                if (speedDemonTimer > 0) BufferString(36, 14, "SPD+", NEON_GREEN);
                if (confusionTimer  > 0) BufferString(72, 14, "CNF", NEON_PINK);
                if (doubleDmgTimer  > 0) BufferString(72, 4,  "2X",  NEON_ORANGE);

                /* HUD: M (melee) and VC on same bottom row */
                {
                    uint16_t meleeCol = (meleeCooldown == 0) ? NEON_ORANGE : 0x7BEF;
                    BufferString(4,  152, "M",  meleeCol);
                    BufferString(18, 152, "VC", vgcActive ? 0xF800 : ST7735_WHITE);
                }

                /* HUD: scan — draw drone positions as cyan dots on screen */
                if (scanTimer > 0) {
                    for (int i = 0; i < NUM_SPRITES; i++) {
                        if (!SpriteList[i].active) continue;
                        /* Project drone world position to minimap pixel position */
                        int mx = (int)(FP_TO_INT(SpriteList[i].x) * SCR_W / MAP_W);
                        int my = (int)(FP_TO_INT(SpriteList[i].y) * SCR_H / MAP_H);
                        /* Draw a 2x2 cyan dot */
                        for (int dy2 = 0; dy2 < 2; dy2++) {
                            for (int dx2 = 0; dx2 < 2; dx2++) {
                                int px = mx + dx2;
                                int py = my + dy2;
                                if (px >= currentChunkXStart && px < currentChunkXEnd
                                    && py >= 0 && py < SCR_H) {
                                    screenBuf[py * 64 + (px - currentChunkXStart)] = NEON_CYAN;
                                }
                            }
                        }
                    }
                }

                /* HUD: reward notification banner — shown for 2 seconds after earn/use */
                if (rewardNotifyTimer > 0) {
                    BufferString(30, 75, rewardName, NEON_YELLOW);
                }

                /* Screen flash border effect on hits, rewards, abilities */
                if (flashTimer > 0) {
                    for (int fy = 0; fy < SCR_H; fy++) {
                        for (int fx = currentChunkXStart; fx < currentChunkXEnd; fx++) {
                            int lx = fx - currentChunkXStart;
                            if (fy < 3 || fy >= SCR_H - 3)
                                screenBuf[fy * 64 + lx] = flashColor;
                        }
                    }
                    /* Left and right border columns */
                    for (int fy = 0; fy < SCR_H; fy++) {
                        if (0   >= currentChunkXStart && 0   < currentChunkXEnd) screenBuf[fy*64+(0  -currentChunkXStart)] = flashColor;
                        if (1   >= currentChunkXStart && 1   < currentChunkXEnd) screenBuf[fy*64+(1  -currentChunkXStart)] = flashColor;
                        if (126 >= currentChunkXStart && 126 < currentChunkXEnd) screenBuf[fy*64+(126-currentChunkXStart)] = flashColor;
                        if (127 >= currentChunkXStart && 127 < currentChunkXEnd) screenBuf[fy*64+(127-currentChunkXStart)] = flashColor;
                    }
                }

                ST7735_DrawBitmap(currentChunkXStart, SCR_H - 1, screenBuf, 64, SCR_H);
            }

            /* Decrement shoot animation and flash timers */
            if(isShooting) {
                shootAnimateTimer--;
                if(shootAnimateTimer == 0) isShooting = 0;
            }
            if(flashTimer > 0) flashTimer--;

            /* Check if all drones are dead — start next round */
            {
                uint32_t anyActive = 0;
                for (int i = 0; i < NUM_SPRITES; i++) {
                    if (SpriteList[i].active) { anyActive = 1; break; }
                }
                if (!anyActive) ResetSurvivalRound();
            }
            break;
                                        
        case STATE_ROUND_SPLASH:
            if(stateChanged){
                ST7735_FillScreen(BLACK);
                ST7735_SetTextColor(NEON_CYAN);
                char roundStr[8];
                char droneStr[16];
                uint32_t cnt = 0;
                for(int i = 0; i < NUM_SPRITES; i++) if(SpriteList[i].active) cnt++;
                sprintf(roundStr, "%d", currentRound);
                sprintf(droneStr, "DRONES: %d", cnt);

                if(myLanguage == English) {
                    ST7735_SetCursor(3, 4);  ST7735_OutString("- ROUND -");
                    ST7735_SetTextColor(NEON_YELLOW);
                    ST7735_SetCursor(7, 7);  ST7735_OutString(roundStr);
                    ST7735_SetTextColor(ST7735_WHITE);
                    ST7735_SetCursor(3, 10); ST7735_OutString(droneStr);
                    ST7735_SetTextColor(NEON_PINK);
                    ST7735_SetCursor(2, 13); ST7735_OutString("GOOD LUCK!");
                } else {
                    /* ラウンド (ra-u-n-do) = Round */
                    ST7735_SetTextColor(NEON_CYAN);
                    Japan_SetCursor(16, 40);
                    const Japan_t round_jp[] = { J_RA, J_U, J_N, J_DO, J_NULL };
                    Japan_OutString(round_jp);
                    ST7735_SetTextColor(NEON_YELLOW);
                    ST7735_SetCursor(7, 7);  ST7735_OutString(roundStr);
                    ST7735_SetTextColor(ST7735_WHITE);
                    ST7735_SetCursor(3, 10); ST7735_OutString(droneStr);
                    ST7735_SetTextColor(NEON_PINK);
                    /* ガンバレ (ga-n-ba-re) = Good luck / Do your best */
                    Japan_SetCursor(16, 120);
                    const Japan_t goodluck_jp[] = { J_GA, J_N, J_BA, J_RE, J_NULL };
                    Japan_OutString(goodluck_jp);
                }
                stateChanged = 0;
            }
            splashTimer--;
            if(splashTimer == 0){
                currentGameState = STATE_PLAYING;
                stateChanged = 1;
            }
            break;

        case STATE_GAME_OVER:
            if(stateChanged){
                ST7735_FillScreen(BLACK);
                char roundOverStr[16];
                sprintf(roundOverStr, "ROUND: %d", currentRound);

                if(myLanguage == English) {
                    ST7735_SetTextColor(NEON_PINK);
                    ST7735_SetCursor(4, 3);  ST7735_OutString("GAME OVER");
                    ST7735_SetTextColor(NEON_YELLOW);
                    ST7735_SetCursor(3, 6);  ST7735_OutString(roundOverStr);
                    ST7735_SetTextColor(ST7735_WHITE);
                    ST7735_SetCursor(3, 9);  ST7735_OutString("FINAL KILLS");
                    ST7735_SetCursor(7, 11); ST7735_OutString(hudString);
                    ST7735_SetTextColor(NEON_CYAN);
                    ST7735_SetCursor(2, 14); ST7735_OutString("B1: RESTART");
                } else {
                    /* ゲームオーバー (ge-mu-cho-o-ba-cho) = Game Over */
                    ST7735_SetTextColor(NEON_PINK);
                    Japan_SetCursor(5, 30);
                    const Japan_t gameover_jp[] = { J_GE, J_MU, J_CHO, J_O, J_BA, J_CHO, J_NULL };
                    Japan_OutString(gameover_jp);
                    ST7735_SetTextColor(NEON_YELLOW);
                    ST7735_SetCursor(3, 6);  ST7735_OutString(roundOverStr);
                    ST7735_SetTextColor(ST7735_WHITE);
                    /* キル (ki-ru) = Kill */
                    Japan_SetCursor(18, 90);
                    const Japan_t kills_jp[] = { J_KI, J_RU, J_NULL };
                    Japan_OutString(kills_jp);
                    ST7735_SetCursor(7, 11); ST7735_OutString(hudString);
                    ST7735_SetTextColor(NEON_CYAN);
                    ST7735_SetCursor(2, 14); ST7735_OutString("B1: RESTART");
                }
                LED_Off(0x00038000);
                stateChanged = 0;
            }
            if (input & 1) {
                currentGameState = STATE_START_SCREEN;
                stateChanged = 1;
            }
            break;
         }
      }
   }
}