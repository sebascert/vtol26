function [F_total, M_total] = calcular_fuerzas_simbolicas_completo()
    % Variables de control (en radianes)
    syms alfa epsilon gamma_izq gamma_der velocidad empuje real
    syms alfa_rud_izq gamma_rud_izq alfa_rud_der gamma_rud_der real
    syms beta_aileron real
    
    % Parámetros físicos
    syms rho area_ala CL_0 CL_alfa CD_0 K_ala real positive
    syms area_cola CL_gamma CD_gamma angulo_cant_cola real positive
    syms area_rud CD_rud CL_rud angulo_cant_rud real positive
    syms area_aileron CL_beta CD_beta real positive
    syms eps_suavizado real positive
    
    % Posiciones de aplicación respecto al CG (marco cuerpo)
    syms r_ala_x r_ala_y r_ala_z real
    syms r_motor_x r_motor_y r_motor_z real
    syms r_aileron_izq_x r_aileron_izq_y r_aileron_izq_z real
    syms r_aileron_der_x r_aileron_der_y r_aileron_der_z real
    syms r_cola_izq_x r_cola_izq_y r_cola_izq_z real
    syms r_cola_der_x r_cola_der_y r_cola_der_z real
    syms r_rud_izq_x r_rud_izq_y r_rud_izq_z real
    syms r_rud_der_x r_rud_der_y r_rud_der_z real
    
    % Momentos directos (ej. reacción de rotores / momentos aerodinámicos no modelados por r x F)
    syms Mx_directo My_directo Mz_directo real
    
    % Presión dinámica
    q_dinamica = (1/2) * rho * velocidad^2;
    
    %% FUERZAS DEL ALA
    CL = CL_0 + CL_alfa * alfa;
    CD = CD_0 + K_ala * CL^2;
    
    % Fuerzas en marco viento
    D_ala = -CD * q_dinamica * area_ala;
    L_ala = CL * q_dinamica * area_ala;
    
    F_viento = [0; D_ala; 0; L_ala];
    
    % Rotación por ángulo de ataque
    q_viento_cuerpo = [cos(alfa/2); 0; -sin(alfa/2); 0];
    q_conjugado_ala = [cos(alfa/2); 0; sin(alfa/2); 0];
    
    F_resultado_ala = quat_mult_sym(quat_mult_sym(q_viento_cuerpo, F_viento), q_conjugado_ala);
    F_ala = F_resultado_ala(2:4);
    
    %% FUERZAS DE ALERONES
    % Alerón izquierdo
    L_aileron_izq = -q_dinamica * CL_beta * beta_aileron * area_aileron;
    D_aileron_izq = -q_dinamica * CD_beta * smooth_abs_sym(beta_aileron, eps_suavizado) * area_aileron;
    
    % Alerón derecho
    L_aileron_der = q_dinamica * CL_beta * beta_aileron * area_aileron;
    D_aileron_der = -q_dinamica * CD_beta * smooth_abs_sym(beta_aileron, eps_suavizado) * area_aileron;
    
    F_aileron_local_izq = [0; D_aileron_izq; 0; L_aileron_izq];
    F_aileron_local_der = [0; D_aileron_der; 0; L_aileron_der];
    
    % Transformación por ángulo de ataque
    F_resultado_aileron_izq = quat_mult_sym(quat_mult_sym(q_viento_cuerpo, F_aileron_local_izq), q_conjugado_ala);
    F_resultado_aileron_der = quat_mult_sym(quat_mult_sym(q_viento_cuerpo, F_aileron_local_der), q_conjugado_ala);
    
    F_aileron_izq = F_resultado_aileron_izq(2:4);
    F_aileron_der = F_resultado_aileron_der(2:4);
    
    %% FUERZAS DEL MOTOR
    F_motor_local = [0; 0; 0; empuje];
    
    % Rotación por ángulo del motor
    q_motor_cuerpo = [cos(epsilon/2); 0; sin(epsilon/2); 0];
    q_conjugado_motor = [cos(epsilon/2); 0; -sin(epsilon/2); 0];
    
    F_resultado_motor = quat_mult_sym(quat_mult_sym(q_motor_cuerpo, F_motor_local), q_conjugado_motor);
    F_motor = F_resultado_motor(2:4);
    
    %% FUERZAS COLA IZQUIERDA
    D_cola_izq = -q_dinamica * CD_gamma * smooth_abs_sym(gamma_izq, eps_suavizado) * area_cola;
    L_cola_izq = q_dinamica * CL_gamma * gamma_izq * area_cola;
    
    F_cola_local_izq = [0; D_cola_izq; 0; L_cola_izq];
    
    % Rotación por ángulo de cant
    q_cola_cuerpo_izq = [cos(-angulo_cant_cola/2); sin(-angulo_cant_cola/2); 0; 0];
    q_conjugado_cola_izq = [cos(-angulo_cant_cola/2); -sin(-angulo_cant_cola/2); 0; 0];
    
    F_resultado_cola_izq = quat_mult_sym(quat_mult_sym(q_cola_cuerpo_izq, F_cola_local_izq), q_conjugado_cola_izq);
    F_cola_izq = F_resultado_cola_izq(2:4);
    
    %% FUERZAS COLA DERECHA
    D_cola_der = -q_dinamica * CD_gamma * smooth_abs_sym(gamma_der, eps_suavizado) * area_cola;
    L_cola_der = q_dinamica * CL_gamma * gamma_der * area_cola;
    
    F_cola_local_der = [0; D_cola_der; 0; L_cola_der];
    
    % Rotación por ángulo de cant
    q_cola_cuerpo_der = [cos(angulo_cant_cola/2); sin(angulo_cant_cola/2); 0; 0];
    q_conjugado_cola_der = [cos(angulo_cant_cola/2); -sin(angulo_cant_cola/2); 0; 0];
    
    F_resultado_cola_der = quat_mult_sym(quat_mult_sym(q_cola_cuerpo_der, F_cola_local_der), q_conjugado_cola_der);
    F_cola_der = F_resultado_cola_der(2:4);
    
    %% FUERZAS RUDDERVATOR IZQUIERDA
    angulo_efectivo_izq = alfa_rud_izq + gamma_rud_izq;
    D_rud_izq = -q_dinamica * CD_rud * smooth_abs_sym(angulo_efectivo_izq, eps_suavizado) * area_rud;
    L_rud_izq = q_dinamica * CL_rud * angulo_efectivo_izq * area_rud;
    
    F_rud_local_izq = [0; D_rud_izq; 0; L_rud_izq];
    
    % Rotación por ángulo de cant
    q_rud_cuerpo_izq = [cos(-angulo_cant_rud/2); sin(-angulo_cant_rud/2); 0; 0];
    q_conjugado_rud_izq = [cos(-angulo_cant_rud/2); -sin(-angulo_cant_rud/2); 0; 0];
    
    F_resultado_rud_izq = quat_mult_sym(quat_mult_sym(q_rud_cuerpo_izq, F_rud_local_izq), q_conjugado_rud_izq);
    F_rud_izq = F_resultado_rud_izq(2:4);
    
    %% FUERZAS RUDDERVATOR DERECHA
    angulo_efectivo_der = alfa_rud_der + gamma_rud_der;
    D_rud_der = -q_dinamica * CD_rud * smooth_abs_sym(angulo_efectivo_der, eps_suavizado) * area_rud;
    L_rud_der = q_dinamica * CL_rud * angulo_efectivo_der * area_rud;
    
    F_rud_local_der = [0; D_rud_der; 0; L_rud_der];
    
    % Rotación por ángulo de cant
    q_rud_cuerpo_der = [cos(angulo_cant_rud/2); sin(angulo_cant_rud/2); 0; 0];
    q_conjugado_rud_der = [cos(angulo_cant_rud/2); -sin(angulo_cant_rud/2); 0; 0];
    
    F_resultado_rud_der = quat_mult_sym(quat_mult_sym(q_rud_cuerpo_der, F_rud_local_der), q_conjugado_rud_der);
    F_rud_der = F_resultado_rud_der(2:4);
    
    %% SUMA TOTAL DE FUERZAS
    % Vector 3x1 con fuerzas totales [Fx; Fy; Fz]
    F_total = simplify(F_ala + F_motor + F_aileron_izq + F_aileron_der + ...
                       F_cola_izq + F_cola_der + F_rud_izq + F_rud_der);
    
    %% MOMENTOS TOTALES RESPECTO AL CG
    r_ala = [r_ala_x; r_ala_y; r_ala_z];
    r_motor = [r_motor_x; r_motor_y; r_motor_z];
    r_aileron_izq = [r_aileron_izq_x; r_aileron_izq_y; r_aileron_izq_z];
    r_aileron_der = [r_aileron_der_x; r_aileron_der_y; r_aileron_der_z];
    r_cola_izq = [r_cola_izq_x; r_cola_izq_y; r_cola_izq_z];
    r_cola_der = [r_cola_der_x; r_cola_der_y; r_cola_der_z];
    r_rud_izq = [r_rud_izq_x; r_rud_izq_y; r_rud_izq_z];
    r_rud_der = [r_rud_der_x; r_rud_der_y; r_rud_der_z];
    
    M_directo = [Mx_directo; My_directo; Mz_directo];
    
    % M_total = sum_i (r_i x F_i) + momentos directos
    M_total = simplify( ...
        cross(r_ala, F_ala) + ...
        cross(r_motor, F_motor) + ...
        cross(r_aileron_izq, F_aileron_izq) + ...
        cross(r_aileron_der, F_aileron_der) + ...
        cross(r_cola_izq, F_cola_izq) + ...
        cross(r_cola_der, F_cola_der) + ...
        cross(r_rud_izq, F_rud_izq) + ...
        cross(r_rud_der, F_rud_der) + ...
        M_directo);
end

function resultado = quat_mult_sym(q1, q2)
    % Multiplicación de cuaterniones: q1 * q2
    q1_0 = q1(1); q1_v = q1(2:4);
    q2_0 = q2(1); q2_v = q2(2:4);
    
    resultado_0 = q1_0 * q2_0 - dot(q1_v, q2_v);
    resultado_v = q1_0 * q2_v + q2_0 * q1_v + cross(q1_v, q2_v);
    
    resultado = [resultado_0; resultado_v];
end

function y = smooth_abs_sym(x, eps_suavizado)
    % Aproximación suave de |x| para evitar no-derivabilidad en x=0.
    y = sqrt(x^2 + eps_suavizado^2);
end

% Ejecutar función
[F, M] = calcular_fuerzas_simbolicas_completo()