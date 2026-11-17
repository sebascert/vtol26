function p = crear_parametros_vtol()
% CREAR_PARAMETROS_VTOL Parametros iniciales para la planta VTOL.
% Ajusta estos valores con tus datos de CAD, identificacion y ensayos.

    % Fisica basica.
    p.g = 9.80665;
    p.rho = 1.225;
    p.m = 1.80;
    p.I = diag([0.040, 0.050, 0.080]);  % [kg m^2]

    % Geometria y aerodinamica.
    p.area_ala = 0.22;
    p.area_cola = 0.03;
    p.area_rud = 0.015;
    p.area_aileron = 0.020;

    p.CL0_ala = 0.25;
    p.CL_alfa_ala = 4.8;
    p.CD0_ala = 0.03;
    p.K_ala = 0.07;

    p.CL_gamma = 3.2;
    p.CD_gamma = 0.20;
    p.CL_rud = 2.6;
    p.CD_rud = 0.22;
    p.CL_beta = 2.2;
    p.CD_beta = 0.14;

    p.angulo_cant_cola = deg2rad(32);
    p.angulo_cant_rud = deg2rad(32);
    p.eps_suavizado = 1e-3;

    % Brazos respecto al CG (marco cuerpo).
    p.r_ala = [0.00; 0.00; 0.00];
    p.r_motor = [0.12; 0.00; 0.00];
    p.r_aileron_izq = [0.02; -0.28; 0.00];
    p.r_aileron_der = [0.02;  0.28; 0.00];
    p.r_cola_izq = [-0.55; -0.18; 0.03];
    p.r_cola_der = [-0.55;  0.18; 0.03];
    p.r_rud_izq = [-0.58; -0.22; 0.05];
    p.r_rud_der = [-0.58;  0.22; 0.05];

    % Momentos directos (reaccion de rotor u otros).
    p.k_reaccion = 0.012;
    p.M_directo = [0; 0; 0];
end
