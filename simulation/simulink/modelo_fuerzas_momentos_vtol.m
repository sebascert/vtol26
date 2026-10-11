function [F_total, M_total] = modelo_fuerzas_momentos_vtol(x, u, p)
% MODELO_FUERZAS_MOMENTOS_VTOL Modelo numerico base en marco cuerpo.
% Este bloque sirve como punto de partida para tu planta en Simulink.
% Puedes reemplazarlo gradualmente con tu modelo aerodinamico completo.

    % Estados utiles.
    vI = x(4:6);
    q = x(7:10);
    q = q / max(norm(q), 1e-9);
    R_BI = quat_a_rot(q);
    vB = R_BI.' * vI;

    % Entradas.
    empuje = u(1);
    epsilon = u(2);
    gamma_izq = u(3);
    gamma_der = u(4);
    alfa_rud_izq = u(5);
    gamma_rud_izq = u(6);
    alfa_rud_der = u(7);
    gamma_rud_der = u(8);
    beta_aileron = u(9);

    % Magnitudes aerodinamicas.
    Va = norm(vB);
    q_dyn = 0.5 * p.rho * Va^2;
    alfa = atan2(-vB(3), max(abs(vB(1)), 1e-3));

    % Ala principal (modelo simple lift/drag).
    CL_ala = p.CL0_ala + p.CL_alfa_ala * alfa;
    CD_ala = p.CD0_ala + p.K_ala * CL_ala^2;
    D_ala = q_dyn * p.area_ala * CD_ala;
    L_ala = q_dyn * p.area_ala * CL_ala;
    F_ala_viento = [-D_ala; 0; L_ala];
    F_ala = rot_y(alfa) * F_ala_viento;

    % Motor principal inclinado en eje y.
    F_motor = rot_y(epsilon) * [0; 0; empuje];

    % Alerones: fuerza opuesta para generar rolido.
    L_ail = q_dyn * p.CL_beta * beta_aileron * p.area_aileron;
    D_ail = q_dyn * p.CD_beta * smooth_abs(beta_aileron, p.eps_suavizado) * p.area_aileron;
    F_ail_izq = [-D_ail; 0; -L_ail];
    F_ail_der = [-D_ail; 0;  L_ail];

    % Cola en V (izquierda/derecha).
    D_cola_izq = q_dyn * p.CD_gamma * smooth_abs(gamma_izq, p.eps_suavizado) * p.area_cola;
    L_cola_izq = q_dyn * p.CL_gamma * gamma_izq * p.area_cola;
    F_cola_izq = rot_x(-p.angulo_cant_cola) * [-D_cola_izq; 0; L_cola_izq];

    D_cola_der = q_dyn * p.CD_gamma * smooth_abs(gamma_der, p.eps_suavizado) * p.area_cola;
    L_cola_der = q_dyn * p.CL_gamma * gamma_der * p.area_cola;
    F_cola_der = rot_x(p.angulo_cant_cola) * [-D_cola_der; 0; L_cola_der];

    % Ruddervators.
    ang_ef_izq = alfa_rud_izq + gamma_rud_izq;
    ang_ef_der = alfa_rud_der + gamma_rud_der;
    D_rud_izq = q_dyn * p.CD_rud * smooth_abs(ang_ef_izq, p.eps_suavizado) * p.area_rud;
    D_rud_der = q_dyn * p.CD_rud * smooth_abs(ang_ef_der, p.eps_suavizado) * p.area_rud;
    L_rud_izq = q_dyn * p.CL_rud * ang_ef_izq * p.area_rud;
    L_rud_der = q_dyn * p.CL_rud * ang_ef_der * p.area_rud;
    F_rud_izq = rot_x(-p.angulo_cant_rud) * [-D_rud_izq; 0; L_rud_izq];
    F_rud_der = rot_x(p.angulo_cant_rud) * [-D_rud_der; 0; L_rud_der];

    % Fuerza total en cuerpo.
    F_total = F_ala + F_motor + F_ail_izq + F_ail_der + F_cola_izq + F_cola_der + F_rud_izq + F_rud_der;

    % Momentos por brazo + momentos directos.
    M_total = ...
        cross(p.r_ala, F_ala) + ...
        cross(p.r_motor, F_motor) + ...
        cross(p.r_aileron_izq, F_ail_izq) + ...
        cross(p.r_aileron_der, F_ail_der) + ...
        cross(p.r_cola_izq, F_cola_izq) + ...
        cross(p.r_cola_der, F_cola_der) + ...
        cross(p.r_rud_izq, F_rud_izq) + ...
        cross(p.r_rud_der, F_rud_der) + ...
        [0; 0; p.k_reaccion * empuje] + ...
        p.M_directo;
end

function y = smooth_abs(x, eps_suavizado)
    y = sqrt(x.^2 + eps_suavizado.^2);
end

function R = rot_x(a)
    R = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
end

function R = rot_y(a)
    R = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
end

function R = quat_a_rot(q)
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
    R = [1-2*(q2^2+q3^2), 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2); ...
         2*(q1*q2+q0*q3), 1-2*(q1^2+q3^2), 2*(q2*q3-q0*q1); ...
         2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), 1-2*(q1^2+q2^2)];
end
