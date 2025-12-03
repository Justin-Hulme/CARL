#include "motor.h"
#include "stm32l476xx.h" /* CMSIS device header */

/* ----------------- Configuration ----------------- */

/* Timer frequency (timer callback ticks per second). Adjust if you want different resolution.
 * We'll use 1000 Hz (1 ms tick) by default. */
#ifndef MOTOR_TIMER_FREQ_HZ
#define MOTOR_TIMER_FREQ_HZ 1000U
#endif

/* Maximum absolute speed we accept (SPS). This is a soft limit — you can change it. */
#ifndef MOTOR_MAX_SPS
#define MOTOR_MAX_SPS 5000 /* reasonable for accumulation scheme; increase if needed */
#endif

/* Pins: PC0..PC3 = motor A, PC4..PC7 = motor B */
#define MOTOR_GPIO_PORT GPIOC
#define MOTOR_A_MASK    0x0FU     /* bits 0..3 */
#define MOTOR_B_MASK    0xF0U     /* bits 4..7 */

/* Half-step sequence (8 steps). The LSB corresponds to IN1 (PCx), next to IN2, etc.
 * For Motor B we will shift these bits left by 4 when writing. */
static const uint8_t half_step_sequence[8] = {
    0b0001, /* step 0 */
    0b0011, /* step 1 */
    0b0010, /* step 2 */
    0b0110, /* step 3 */
    0b0100, /* step 4 */
    0b1100, /* step 5 */
    0b1000, /* step 6 */
    0b1001  /* step 7 */
};

/* ----------------- Internal state ----------------- */
typedef struct {
    volatile int32_t commanded_sps;    /* signed requested speed (SPS) */
    uint32_t abs_sps;                  /* cached absolute speed */
    uint32_t acc;                      /* accumulator for fractional stepping (0..TIMER_FREQ-1) */
    uint8_t step_index;                /* 0..7 current half-step index */
    bool enabled;                      /* true if stepping requested */
} motor_state_t;

static motor_state_t motors[MOTOR_COUNT];

/* System-provided timer tick frequency - set at init */
static uint32_t g_timer_freq_hz = MOTOR_TIMER_FREQ_HZ;

/* ----------------- Helpers ----------------- */

/* Update outputs for motor A and B (writes to PC0..PC7)
 * We compute the two 4-bit patterns and write them with a single atomic ODR write.
 */
static inline void write_motor_outputs(uint8_t outA, uint8_t outB)
{
    uint32_t odr = MOTOR_GPIO_PORT->ODR;
    /* clear relevant bits then set */
    odr &= ~(MOTOR_A_MASK | MOTOR_B_MASK);
    odr |= ((uint32_t)(outA & 0x0F)) | ((uint32_t)(outB & 0x0F) << 4);
    MOTOR_GPIO_PORT->ODR = odr;
}

/* Safe function to step a single motor one half-step in direction dir (dir = +1 or -1). */
static inline void motor_do_step(motor_id_t id, int dir)
{
    motor_state_t *m = &motors[id];
    if (dir >= 0) {
        m->step_index = (m->step_index + 1) & 0x07;
    } else {
        m->step_index = (m->step_index - 1) & 0x07;
    }
}

/* ----------------- API Implementation ----------------- */

void motor_init(uint32_t system_core_clock_hz)
{
    /* Initialize state */
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        motors[i].commanded_sps = 0;
        motors[i].abs_sps = 0;
        motors[i].acc = 0;
        motors[i].step_index = 0;
        motors[i].enabled = false;
    }

    g_timer_freq_hz = MOTOR_TIMER_FREQ_HZ;

    /* --- Enable GPIOC clock --- */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
    (void)RCC->AHB2ENR; /* small delay */

    /* Configure PC0..PC7 as outputs (push-pull, no pull) */
    /* MODER: 00=input 01=output for each pin */
    /* Clear and set */
    for (int pin = 0; pin <= 7; ++pin) {
        MOTOR_GPIO_PORT->MODER &= ~(3U << (pin * 2));
        MOTOR_GPIO_PORT->MODER |=  (1U << (pin * 2)); /* output */
    }
    /* OTYPER default (push-pull) is fine */
    MOTOR_GPIO_PORT->OSPEEDR &= ~0xFFFFFFFFU; /* low speed is fine */
    MOTOR_GPIO_PORT->PUPDR &= ~0xFFFFFFFFU; /* no pulls */

    /* Set outputs low initially */
    MOTOR_GPIO_PORT->ODR &= ~(MOTOR_A_MASK | MOTOR_B_MASK);

    /* --- Configure TIM2 as periodic update interrupt at g_timer_freq_hz --- */
    /* Enable TIM2 clock */
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    (void)RCC->APB1ENR1;

    /* Calculate prescaler and ARR to get update frequency = g_timer_freq_hz
     *
     * Timer counter clock (TIM_CLK) on STM32L4 typically equals SystemCoreClock
     * if APB prescalers are 1. If your timers run at different freq adjust accordingly.
     *
     * We'll compute:
     *   prescaler = (system_core_clock_hz / g_timer_freq_hz) - 1
     *   ARR = 1  (so update at 1 tick per prescaled period)
     *
     * If system_core_clock_hz is not an integer multiple of desired frequency,
     * the nearest prescaler will be used.
     */
    uint32_t presc = 0;
    if (system_core_clock_hz > g_timer_freq_hz) {
        presc = (system_core_clock_hz / g_timer_freq_hz) - 1U;
    } else {
        presc = 0;
    }

    TIM2->CR1 = 0;                 /* reset control */
    TIM2->PSC = presc;             /* prescaler */
    TIM2->ARR = 1U;                /* auto-reload -> update event when counter reaches 1 */
    TIM2->EGR = TIM_EGR_UG;        /* generate update to load PSC/ARR */
    TIM2->DIER = TIM_DIER_UIE;     /* enable update interrupt */
    TIM2->CR1 |= TIM_CR1_CEN;      /* enable counter */

    /* Enable TIM2 IRQ in NVIC */
    NVIC_SetPriority(TIM2_IRQn, 3);
    NVIC_EnableIRQ(TIM2_IRQn);
}

void motor_set_speed(motor_id_t id, int32_t steps_per_sec)
{
    if (id < 0 || id >= MOTOR_COUNT) return;

    /* clamp */
    if (steps_per_sec > (int32_t)MOTOR_MAX_SPS) steps_per_sec = MOTOR_MAX_SPS;
    if (steps_per_sec < -(int32_t)MOTOR_MAX_SPS) steps_per_sec = -(int32_t)MOTOR_MAX_SPS;

    __disable_irq();
    motors[id].commanded_sps = steps_per_sec;
    if (steps_per_sec == 0) {
        motors[id].abs_sps = 0;
        motors[id].enabled = false;
    } else {
        motors[id].enabled = true;
        motors[id].abs_sps = (uint32_t)(steps_per_sec < 0 ? -steps_per_sec : steps_per_sec);
        if (motors[id].abs_sps > (uint32_t)MOTOR_MAX_SPS) motors[id].abs_sps = MOTOR_MAX_SPS;
    }
    __enable_irq();
}

int32_t motor_get_speed(motor_id_t id)
{
    if (id < 0 || id >= MOTOR_COUNT) return 0;
    return motors[id].commanded_sps;
}

void motor_stop_all(void)
{
    __disable_irq();
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        motors[i].commanded_sps = 0;
        motors[i].abs_sps = 0;
        motors[i].enabled = false;
        motors[i].acc = 0;
    }
    /* clear outputs */
    MOTOR_GPIO_PORT->ODR &= ~(MOTOR_A_MASK | MOTOR_B_MASK);
    __enable_irq();
}

/* This function is called from TIM2 IRQ. It runs in interrupt context.
 * It uses an accumulator approach: each tick we add abs_sps to accumulator,
 * when accumulator >= timer_freq we perform one half-step and subtract timer_freq.
 * This allows steps/sec > timer frequency (we'll get multiple steps per tick when needed).
 */
void motor_timer_irq_handler(void)
{
    /* Check update interrupt flag */
    if ((TIM2->SR & TIM_SR_UIF) == 0) return;
    TIM2->SR &= ~TIM_SR_UIF; /* clear flag */

    uint8_t outA = 0;
    uint8_t outB = 0;

    /* motor A */
    if (motors[MOTOR_A].enabled && motors[MOTOR_A].abs_sps != 0) {
        motors[MOTOR_A].acc += motors[MOTOR_A].abs_sps;
        while (motors[MOTOR_A].acc >= g_timer_freq_hz) {
            motors[MOTOR_A].acc -= g_timer_freq_hz;
            /* direction = sign of commanded_sps */
            motor_do_step(MOTOR_A, (motors[MOTOR_A].commanded_sps >= 0) ? +1 : -1);
        }
        outA = half_step_sequence[motors[MOTOR_A].step_index & 0x07];
    } else {
        outA = 0;
    }

    /* motor B */
    if (motors[MOTOR_B].enabled && motors[MOTOR_B].abs_sps != 0) {
        motors[MOTOR_B].acc += motors[MOTOR_B].abs_sps;
        while (motors[MOTOR_B].acc >= g_timer_freq_hz) {
            motors[MOTOR_B].acc -= g_timer_freq_hz;
            motor_do_step(MOTOR_B, (motors[MOTOR_B].commanded_sps >= 0) ? +1 : -1);
        }
        outB = half_step_sequence[motors[MOTOR_B].step_index & 0x07];
    } else {
        outB = 0;
    }

    /* write combined outputs */
    write_motor_outputs(outA, outB);
}

/* TIM2 IRQ wrapper for CMSIS IRQ name */
void TIM2_IRQHandler(void)
{
    motor_timer_irq_handler();
}
