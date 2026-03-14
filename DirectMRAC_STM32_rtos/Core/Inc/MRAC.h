

/**
 * @file MRAC.h
 * @brief Model Reference Adaptive Control (MRAC) implementation for STM32 RTOS project.
 *
 * Developed for educational and research purposes.
 *
 * @author Dr. Bharat Verma
 * @note Assistant Professor, The LNMIIT, Jaipur, India
 * @note ORCID: https://orcid.org/0000-0001-7600-7872
 * @note GitHub: https://github.com/vkbharatv
 *
 * Description:
 *   This header defines structures and functions for implementing a Direct MRAC algorithm.
 *   The MRAC adapts controller gains online to minimize tracking error between plant and reference model.
 *   Includes anti-windup logic for actuator saturation and a simple first-order reference model.
 *
 * Structures:
 *   - TF_model: Represents a first-order transfer function model for reference dynamics.
 *   - MRAC: Contains adaptive gains, tracking error, plant/reference outputs, and control input.
 *
 * Functions:
 *   - TF_model_init: Initializes the reference model parameters.
 *   - TF_model_step: Steps the reference model forward given an input.
 *   - MRAC_init: Initializes MRAC parameters and links reference model.
 *   - MRAC_reset: Resets MRAC states and adaptive gains.
 *   - MRAC_set_reference: Updates the reference input.
 *   - MRAC_update: Performs MRAC adaptation and computes control input with anti-windup.
 *
 * Usage:
 *   - Initialize TF_model and MRAC structures.
 *   - Call MRAC_update() in control loop with plant output.
 *   - Use MRAC_set_reference() to update reference input as needed.
 *
 * Features:
 *   - Adaptive gain update using gradient descent.
 *   - Anti-windup for actuator saturation.
 *   - Modular reference model for flexible dynamics.
 *
 * Applications:
 *   - Adaptive control of uncertain systems.
 *   - Real-time embedded control for STM32.
 *   - Educational demonstration of MRAC principles.
 */
#ifndef MRAC_H
#define MRAC_H
typedef struct
{
    float data;
    float prev_data;
    float tau;
    float alpha;
    float dt;
} TF_model;

typedef struct
{
  float gamma; // Adaptation gain
  float Kx_hat; // Estimated plant gain
  float Kr_hat; // Reference model gain
  float e;      // Tracking error
  float x;      // Plant output
  float xm;     // Reference model output
  float r;      // Reference input
  float u;      // Control input
  float dt;
  float umax;
    float umin;
  TF_model *reference_model; // Optional: can be used to implement a specific reference model
  
} MRAC;



float TF_model_init(TF_model *model,float tau,float dt)
    {
      model->data = 0;
      model->prev_data = 0;
      model->tau = tau;
      model->dt = dt;
      model->alpha = dt / (tau + dt);
      return model->data;
}
float TF_model_step(TF_model *model, float input) {
        model->data = model->alpha * input + (1 - model->alpha) * model->prev_data;
        model->prev_data = model->data;
      return model->data;}

void MRAC_init(MRAC *mrac, TF_model *plant_ref, float r, float gamma, float Kx_hat_init, float Kr_hat_init,float dt, float umax, float umin)
    {
      mrac->gamma = gamma;
      mrac->Kx_hat = Kx_hat_init;
      mrac->Kr_hat = Kr_hat_init;
      mrac->r = r;
      mrac->dt = dt;
      mrac->e = 0.0f;
      mrac->x = 0.0f;
      mrac->xm = 0.0f;
      mrac->u = 0.0f;
      mrac->reference_model = plant_ref; // Store the reference model
      mrac->umax = umax;
      mrac->umin = umin;
}

void MRAC_reset(MRAC *mrac)
    {
      mrac->e = 0.0f;
      mrac->x = 0.0f;
      mrac->xm = 0.0f;
      mrac->u = 0.0f;
      mrac->Kx_hat = 0.0f;
      mrac->Kr_hat = 0.0f;
}

void MRAC_set_reference(MRAC *mrac, float r) { mrac->r = r; }


float MRAC_update(MRAC *mrac, float x) {
        mrac->x = x;
        mrac->xm = TF_model_step(mrac->reference_model, mrac->r); // Update reference model output
        mrac->e = mrac->x - mrac->xm; // Tracking error

        mrac->Kx_hat += (-mrac->gamma * mrac->e * mrac->x ) * mrac->dt;
        // Update Kr_hat with sigma modification
        mrac->Kr_hat += (-mrac->gamma * mrac->e * mrac->r) * mrac->dt;
        // mrac->Kx_hat = -22.0;
        // mrac->Kr_hat = 26.0;
        mrac->u = mrac->Kx_hat * mrac->x +
                  mrac->Kr_hat * mrac->r; // Compute control input
        if (mrac->u > mrac->umax) {
          mrac->u = mrac->umax; // Saturate control input
          // Anti-windup: reverse update with sigma modification
          mrac->Kx_hat += (mrac->gamma * mrac->e * mrac->x ) * mrac->dt;
          mrac->Kr_hat += (mrac->gamma * mrac->e * mrac->r ) * mrac->dt;
        } else if (mrac->u < mrac->umin) {
          // Anti-windup: reverse update with sigma modification
          mrac->Kx_hat += (mrac->gamma * mrac->e * mrac->x ) * mrac->dt;
          mrac->Kr_hat += (mrac->gamma * mrac->e * mrac->r ) * mrac->dt;
          mrac->u = mrac->umin; // Saturate control input
        }
        return mrac->u;
}

#endif /* MRAC_H */