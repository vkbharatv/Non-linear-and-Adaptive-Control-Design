/**
 * @file controller.c
 * @brief PID controller implementation with filter for STM32 RTOS project.
 *
 * Developed for educational and research purposes.
 *
 * @author Dr. Bharat Verma
 * @note Assistant Professor, The LNMIIT, Jaipur, India
 * @note ORCID: https://orcid.org/0000-0001-7600-7872
 * @note GitHub: https://github.com/vkbharatv
 * Description: PID controller implementation with filter for STM32.
 */

#include "controller.h"

void filter_init(filter *f, float tau) {
    f->tau = tau;
    f->dt = 0.01f; // Assuming a fixed time step of 10 ms
    f->alpha = f->dt / (f->tau + f->dt);
    f->data = 0.0f;
    f->prev_data = 0.0f;
}

float apply_filter(filter *f, float new_data) {
    f->data = f->alpha * new_data + (1.0f - f->alpha) * f->prev_data;
    f->prev_data = f->data;
    return f->data;
}

float reset_filter(filter *f) {
    f->data = 0.0f;
    f->prev_data = 0.0f;
    return f->data;
}

void reset_controller(controlData *c) {
    c->e = 0.0f;
    c->e_i = 0.0f;
    c->e_d = 0.0f;
    c->e_prev = 0.0f;
    c->e_dprev = 0.0f;
    c->u = 0.0f;
    c->derivative_filter.data = 0.0f;
    c->derivative_filter.prev_data = 0.0f;
}

void init_controller(controlData *c, float Kp, float Ki, float Kd, float N, float umax, float umin, float dt) {
    c->Kp = Kp;
    c->Ki = Ki;
    c->Kd = Kd;
    c->N = N;
    c->umax = umax;
    c->umin = umin;
    c->dt = dt;
    c->e = 0.0f;
    c->e_i = 0.0f;
    c->e_d = 0.0f;
    c->e_prev = 0.0f;
    c->e_dprev = 0.0f;
    c->u = 0.0f;
    c->dalpha = N * dt / (1.0f + N * dt);
    c->derivative_filter.data = 0.0f;
    c->derivative_filter.prev_data = 0.0f;
    c->derivative_filter.alpha = c->dalpha; // Tau = 10ms for derivative filter
}

void update_gains(controlData *c, float Kp, float Ki, float Kd, float N) {
    c->Kp = Kp;
    c->Ki = Ki;
    c->Kd = Kd;
    c->N = N;
    c->dalpha = N * c->dt / (1.0f + N * c->dt);
}

float control_apply(controlData *c, float error) {
    c->e = error;
    c->e_i += c->e * c->dt; // Integral with anti-windup
    c->e_d = (c->e - c->e_prev) / c->dt; // Simplified derivative
    c->e_d = apply_filter(&c->derivative_filter, c->e_d);
    c->e_prev = c->e;
    c->e_dprev = c->e_d;
    c->u = c->Kp * c->e + c->Ki * c->e_i + c->Kd * c->e_d;
    if (c->u > c->umax) {
        c->u = c->umax;
        c->e_i -= c->e * c->dt; // Anti-windup
    } else if (c->u < c->umin) {
        c->u = c->umin;
        c->e_i -= c->e * c->dt; // Anti-windup
    }
    return c->u;
}
