#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

/*
 * motor.h
 *
 * API for driving two 28BYJ-48 steppers using PC0..PC7 and a hardware timer (TIM2).
 *
 * Pin mapping:
 *   Motor A -> PC0 (IN1), PC1 (IN2), PC2 (IN3), PC3 (IN4)
 *   Motor B -> PC4 (IN1), PC5 (IN2), PC6 (IN3), PC7 (IN4)
 *
 * Speed units:
 *   - The public API uses signed steps-per-second (SPS).
 *   - Positive SPS = forward (index increasing).
 *   - Negative SPS = reverse (index decreasing).
 *   - SPS = 0 stops the motor (no stepping).
 *
 * Notes:
 *   - Call motor_init(SystemCoreClock) once at startup (before enabling interrupts).
 *   - Call TIM2_IRQHandler or motor_timer_irq_handler() from your TIM2 update ISR.
 *
 * Example:
 *   motor_init(SystemCoreClock);
 *   motor_set_speed(MOTOR_A, 400);   // 400 steps/s forward
 *   motor_set_speed(MOTOR_B, -200);  // 200 steps/s reverse
 */

typedef enum {
    MOTOR_A = 0,
    MOTOR_B = 1,
    MOTOR_COUNT = 2
} motor_id_t;

/* Initialize motor GPIOs and the timer. Pass your SystemCoreClock (Hz). */
void motor_init(uint32_t system_core_clock_hz);

/* Set speed in steps-per-second (signed). Use 0 to stop. */
void motor_set_speed(motor_id_t id, int32_t steps_per_sec);

/* Emergency stop (immediately stops both motors) */
void motor_stop_all(void);

/* Must be called from TIM2 update IRQ handler (or call TIM2 IRQ directly) */
void motor_timer_irq_handler(void);

/* Optional: returns current commanded speed (signed SPS) */
int32_t motor_get_speed(motor_id_t id);

#endif /* MOTOR_H */
