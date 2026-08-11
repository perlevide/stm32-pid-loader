/* pid_config.h
 * ---------------------------------------------------------------------------
 * Tuning constants for the 3-sensor line follower.
 *
 * The desktop tool (stm32_pid_flasher.py) rewrites the three PID_K* lines
 * below by regex. Keep them on one line each, in the form:
 *
 *     #define PID_KP   1.0000f
 *
 * Do not add trailing comments on those three lines.
 * ---------------------------------------------------------------------------
 */
#ifndef PID_CONFIG_H
#define PID_CONFIG_H

/* --- Patched by the flasher tool -------------------------------------- */
#define PID_KP   1234.0000f
#define PID_KI   32.0000f
#define PID_KD   3.0000f
/* ---------------------------------------------------------------------- */

/* Base forward speed, 0..1000 (matches the PWM period below).
 * Start low (250-350) while tuning Kp, raise once the car tracks cleanly. */
#define BASE_SPEED        300

/* Hard cap on a single wheel's PWM. Leave headroom so the correction term
 * can still bite at full speed. */
#define MAX_SPEED         1000

/* Integral windup clamp, in the same units as the PID output. */
#define I_CLAMP           400.0f

/* Control loop period in milliseconds. PID gains are scaled against this,
 * so changing it changes the effective Ki and Kd. */
#define LOOP_MS           5

/* Multiplier applied to the last known error when all three sensors lose
 * the line, so the car sweeps back toward where the line went. */
#define LOST_LINE_GAIN    3.0f

/* How many consecutive lost-line loops before the car stops.
 * 400 loops x 5 ms = 2 s. Set to 0 to never stop. */
#define LOST_LINE_TIMEOUT 400

/* Set to 1 if a sensor reads LOW when it is over the line (most IR modules
 * with a black line on white floor), 0 if it reads HIGH. */
#define LINE_ACTIVE_LOW   1

#endif /* PID_CONFIG_H */
