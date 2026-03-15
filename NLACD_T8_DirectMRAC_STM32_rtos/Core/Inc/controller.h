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
 // Filter Design

#ifndef CONTROLLER_H
#define CONTROLLER_H
#ifdef __cplusplus
extern "C"
 {
#endif

     typedef struct
     {
         float data;
         float prev_data;
         float tau;
         float alpha;
         float dt;
     } filter;

     void filter_init(filter * f, float tau);
     float apply_filter(filter * f, float new_data);
     float reset_filter(filter * f);

     typedef struct
     {
         float e, e_i, e_d, e_prev, e_dprev;
         float Kp, Ki, Kd, N, dalpha;
         float u;
         float umax, umin;
         float dt;
         filter derivative_filter;
         void (*update_gains)(struct controlData *c, float Kp, float Ki, float Kd, float N);
         float (*apply)(struct controlData *c, float error);
     } controlData;

     void reset_controller(controlData * c);
     void init_controller(controlData * c, float Kp, float Ki, float Kd, float N, float umax, float umin, float dt);
     void update_gains(controlData * c, float Kp, float Ki, float Kd, float N);
     float control_apply(controlData * c, float error);

#ifdef __cplusplus
 }
#endif

#endif /* CONTROLLER_H */
