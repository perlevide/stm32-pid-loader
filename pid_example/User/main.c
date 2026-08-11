/* main.c — bare-metal line follower for STM32F103C8T6 (Blue Pill)
 * ---------------------------------------------------------------------------
 * No HAL, no CubeMX, no external headers. Builds with arm-none-eabi-gcc into a
 * single .hex you can flash straight to the board.
 *
 * Wiring:
 *   Left sensor    PB12        PWMA (left pair)   PA6  (TIM3_CH1)
 *   Center sensor  PB13        AIN1/AIN2          PB0 / PB1
 *   Right sensor   PB14        PWMB (right pair)  PA7  (TIM3_CH2)
 *   (inputs, pull-up)          BIN1/BIN2          PB10 / PB11
 *                              STBY               PB5
 *
 * Clock: HSE 8 MHz crystal -> PLL x9 -> 72 MHz. TIM3 runs at 72 MHz,
 * prescaled to 1 MHz, period 1000 -> 1 kHz PWM, duty range 0..1000.
 * ---------------------------------------------------------------------------
 */

#include <stdint.h>
#include "pid_config.h"

/* ---- register map (only what we touch) -------------------------------- */
#define REG(addr)       (*(volatile uint32_t *)(addr))

#define RCC_CR          REG(0x40021000)
#define RCC_CFGR        REG(0x40021004)
#define RCC_APB2ENR     REG(0x40021018)
#define RCC_APB1ENR     REG(0x4002101C)
#define FLASH_ACR       REG(0x40022000)

#define GPIOA_CRL       REG(0x40010800)
#define GPIOB_CRL       REG(0x40010C00)
#define GPIOB_CRH       REG(0x40010C04)
#define GPIOB_IDR       REG(0x40010C08)
#define GPIOB_ODR       REG(0x40010C0C)
#define GPIOB_BSRR      REG(0x40010C10)

#define TIM3_CR1        REG(0x40000400)
#define TIM3_EGR        REG(0x40000414)
#define TIM3_CCMR1      REG(0x40000418)
#define TIM3_CCER       REG(0x40000420)
#define TIM3_PSC        REG(0x40000428)
#define TIM3_ARR        REG(0x4000042C)
#define TIM3_CCR1       REG(0x40000434)
#define TIM3_CCR2       REG(0x40000438)

#define STK_CTRL        REG(0xE000E010)
#define STK_LOAD        REG(0xE000E014)
#define STK_VAL         REG(0xE000E018)

/* ---- pins -------------------------------------------------------------- */
#define PIN_AIN1   (1u << 0)    /* PB0  */
#define PIN_AIN2   (1u << 1)    /* PB1  */
#define PIN_STBY   (1u << 5)    /* PB5  */
#define PIN_BIN1   (1u << 10)   /* PB10 */
#define PIN_BIN2   (1u << 11)   /* PB11 */
#define PIN_SENS_L (1u << 12)   /* PB12 */
#define PIN_SENS_C (1u << 13)   /* PB13 */
#define PIN_SENS_R (1u << 14)   /* PB14 */

/* ---- millisecond tick -------------------------------------------------- */
static volatile uint32_t ms_ticks;

void SysTick_Handler(void) { ms_ticks++; }

static uint32_t millis(void) { return ms_ticks; }

static void delay_ms(uint32_t n)
{
    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < n) { }
}

/* ---- clock: 8 MHz HSE -> 72 MHz ---------------------------------------- */
static void clock_init(void)
{
    /* Skip the PLL dance if something already put us on it — Keil's stock
     * device startup calls SystemInit(), which sets 72 MHz before main().
     * Reconfiguring the PLL while running from it would hang. */
    if (((RCC_CFGR >> 2) & 3u) != 2u) {
        RCC_CR |= (1u << 16);                   /* HSEON                    */
        while (!(RCC_CR & (1u << 17))) { }      /* wait HSERDY              */

        FLASH_ACR = (FLASH_ACR & ~7u) | 2u | (1u << 4);  /* 2 WS, prefetch  */

        RCC_CFGR &= ~(0xFu << 18);
        RCC_CFGR |= (7u << 18);                 /* PLLMUL = x9              */
        RCC_CFGR |= (1u << 16);                 /* PLLSRC = HSE             */
        RCC_CFGR |= (4u << 8);                  /* APB1 = HCLK/2 = 36 MHz   */

        RCC_CR |= (1u << 24);                   /* PLLON                    */
        while (!(RCC_CR & (1u << 25))) { }      /* wait PLLRDY              */

        RCC_CFGR = (RCC_CFGR & ~3u) | 2u;       /* SYSCLK = PLL             */
        while (((RCC_CFGR >> 2) & 3u) != 2u) { }
    }

    STK_LOAD = 72000u - 1u;                     /* 1 ms tick                */
    STK_VAL  = 0;
    STK_CTRL = 7u;                              /* core clock, IRQ, enable  */
}

/* ---- GPIO + PWM -------------------------------------------------------- */
static void gpio_init(void)
{
    RCC_APB2ENR |= (1u << 2) | (1u << 3);       /* GPIOA, GPIOB clocks      */
    RCC_APB1ENR |= (1u << 1);                   /* TIM3 clock               */

    /* PA6, PA7: alternate function push-pull, 50 MHz (0xB per nibble) */
    GPIOA_CRL = (GPIOA_CRL & 0x00FFFFFFu) | 0xBB000000u;

    /* PB0, PB1: output push-pull 50 MHz (0x3). PB5: same. */
    GPIOB_CRL = (GPIOB_CRL & 0xFF0FFF00u) | 0x00300033u;

    /* PB10, PB11: output push-pull. PB12..PB14: input with pull (0x8). */
    GPIOB_CRH = (GPIOB_CRH & 0xF00000FFu) | 0x08883300u;

    /* Pull-ups on the three sensor inputs (ODR high selects pull-up). */
    GPIOB_BSRR = PIN_SENS_L | PIN_SENS_C | PIN_SENS_R;
}

static void pwm_init(void)
{
    TIM3_PSC   = 71;                            /* 72 MHz -> 1 MHz          */
    TIM3_ARR   = MAX_SPEED;                     /* 1 kHz, duty 0..MAX_SPEED */
    TIM3_CCMR1 = 0x6868u;                       /* PWM mode 1 + preload, CH1+CH2 */
    TIM3_CCER  = 0x0011u;                       /* enable CH1, CH2 outputs  */
    TIM3_CCR1  = 0;
    TIM3_CCR2  = 0;
    TIM3_EGR   = 1u;                            /* load the preloads        */
    TIM3_CR1   = (1u << 7) | 1u;                /* ARPE + counter enable    */
}

/* ---- sensors ----------------------------------------------------------- */
static int sensor_on_line(uint32_t pin)
{
    uint32_t level = GPIOB_IDR & pin;
#if LINE_ACTIVE_LOW
    return level == 0u;
#else
    return level != 0u;
#endif
}

/* Negative error = line is left of center, positive = right.
 * Returns 0 when all three sensors have lost the line. */
static int read_error(float *error)
{
    uint32_t pattern = 0;
    if (sensor_on_line(PIN_SENS_L)) pattern |= 4u;
    if (sensor_on_line(PIN_SENS_C)) pattern |= 2u;
    if (sensor_on_line(PIN_SENS_R)) pattern |= 1u;

    switch (pattern) {
        case 2u: *error =  0.0f; return 1;   /* 010 centered             */
        case 6u: *error = -1.0f; return 1;   /* 110 drifting right       */
        case 4u: *error = -2.0f; return 1;   /* 100 well right           */
        case 3u: *error =  1.0f; return 1;   /* 011 drifting left        */
        case 1u: *error =  2.0f; return 1;   /* 001 well left            */
        case 7u: *error =  0.0f; return 1;   /* 111 intersection, go on  */
        case 5u: *error =  0.0f; return 1;   /* 101 fork or noise        */
        default: return 0;                   /* 000 line lost            */
    }
}

/* ---- motors ------------------------------------------------------------ */
static int clamp_speed(int v)
{
    if (v >  MAX_SPEED) return  MAX_SPEED;
    if (v < -MAX_SPEED) return -MAX_SPEED;
    return v;
}

/* Signed speed: negative reverses that side, which lets the car pivot through
 * corners tighter than the base speed alone would allow. */
static void set_side(uint32_t in1, uint32_t in2, volatile uint32_t *ccr, int speed)
{
    speed = clamp_speed(speed);
    if (speed >= 0) {
        GPIOB_BSRR = in1 | (in2 << 16);
    } else {
        GPIOB_BSRR = in2 | (in1 << 16);
        speed = -speed;
    }
    *ccr = (uint32_t)speed;
}

static void drive(int left, int right)
{
    set_side(PIN_AIN1, PIN_AIN2, &TIM3_CCR1, left);
    set_side(PIN_BIN1, PIN_BIN2, &TIM3_CCR2, right);
}

static void motors_off(void)
{
    TIM3_CCR1 = 0;
    TIM3_CCR2 = 0;
    GPIOB_BSRR = (PIN_AIN1 | PIN_AIN2 | PIN_BIN1 | PIN_BIN2) << 16;
}

/* ---- main -------------------------------------------------------------- */
int main(void)
{
    const float dt = (float)LOOP_MS / 1000.0f;
    float integral = 0.0f, prev_error = 0.0f, last_error = 0.0f;
    uint32_t lost_count = 0;
    uint32_t next;

    clock_init();
    gpio_init();
    pwm_init();

    motors_off();
    GPIOB_BSRR = PIN_STBY;          /* wake the TB6612 */
    delay_ms(1000);                 /* time to put the car down */

    next = millis();
    for (;;) {
        float error, derivative, output;

        if (read_error(&error)) {
            lost_count = 0;
            last_error = error;
        } else {
            /* Line gone: steer harder toward where it was last seen. */
            error = last_error * LOST_LINE_GAIN;
            lost_count++;
#if LOST_LINE_TIMEOUT > 0
            if (lost_count > LOST_LINE_TIMEOUT) {
                motors_off();
                for (;;) { }        /* stop for good */
            }
#endif
        }

        integral += error * dt;
        if (integral >  I_CLAMP) integral =  I_CLAMP;
        if (integral < -I_CLAMP) integral = -I_CLAMP;

        derivative = (error - prev_error) / dt;
        prev_error = error;

        output = (PID_KP * error) + (PID_KI * integral) + (PID_KD * derivative);

        drive(BASE_SPEED + (int)output, BASE_SPEED - (int)output);

        next += LOOP_MS;
        while ((int32_t)(millis() - next) < 0) { }
    }
}
