//Esto representa la salida del PSO, los actuadores que quiero enviar

#pragma once

struct ActuatorCommand
{
    float omega_i;
    float omega_d;
    float omega_t;

    float epsilon_i;
    float epsilon_d;

    float gamma_i;
    float gamma_d;

    float beta_i;
    float beta_d;
};