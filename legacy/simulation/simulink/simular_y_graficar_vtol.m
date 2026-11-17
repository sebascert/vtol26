function simular_y_graficar_vtol(nombre_modelo, u_constante, tiempo_final)
% SIMULAR_Y_GRAFICAR_VTOL Ejecuta simulacion y grafica variables principales.
%
% Uso:
%   simular_y_graficar_vtol
%   simular_y_graficar_vtol('planta_vtol')
%   simular_y_graficar_vtol('planta_vtol', [17.7;0;0;0;0;0;0;0;0], 20)

    if nargin < 1 || isempty(nombre_modelo)
        nombre_modelo = 'planta_vtol';
    end
    if nargin < 2 || isempty(u_constante)
        % Entrada base de prueba: empuje cercano a peso, resto en cero.
        p = crear_parametros_vtol();
        u_constante = [p.m * p.g; 0; 0; 0; 0; 0; 0; 0; 0];
    end
    if nargin < 3 || isempty(tiempo_final)
        tiempo_final = 20;
    end

    if ~bdIsLoaded(nombre_modelo)
        construir_planta_vtol(nombre_modelo);
    end

    % Asegurar que el modelo tiene los bloques de guardado esperados.
    if ~tiene_bloques_salida(nombre_modelo)
        warning(['El modelo no contiene bloques To Workspace esperados. ' ...
                 'Se reconstruira para sincronizar la plantilla.']);
        construir_planta_vtol(nombre_modelo);
    end

    % Crea/actualiza bloque de entrada constante 9x1.
    bloque_u_const = [nombre_modelo '/u_constante'];
    if ~bdIsLoaded(nombre_modelo)
        open_system(nombre_modelo);
    end
    if isempty(find_system(nombre_modelo, 'SearchDepth', 1, 'Name', 'u_constante'))
        add_block('simulink/Sources/Constant', bloque_u_const, ...
            'Value', 'u_constante_vtol', ...
            'OutDataTypeStr', 'double', ...
            'Position', [20 40 140 70]);
        try
            delete_line(nombre_modelo, 'u_control/1', 'DinamicaVTOL/2');
        catch
        end
        add_line(nombre_modelo, 'u_constante/1', 'DinamicaVTOL/2', 'autorouting', 'on');
    end

    assignin('base', 'u_constante_vtol', u_constante);
    set_param(nombre_modelo, 'StopTime', num2str(tiempo_final));

    % Ejecutar simulacion y capturar salida.
    simOut = sim(nombre_modelo, 'ReturnWorkspaceOutputs', 'on');

    % Leer resultados (prioriza SimulationOutput, luego workspace base).
    x_ws = obtener_variable_salida(simOut, 'estado_x_ws');
    F_ws = obtener_variable_salida(simOut, 'fuerza_body_ws');
    M_ws = obtener_variable_salida(simOut, 'momento_body_ws');

    if isempty(x_ws) || isempty(F_ws) || isempty(M_ws)
        error(['No se pudieron recuperar salidas de simulacion (estado/fuerza/momento). ' ...
               'Reconstruye el modelo con construir_planta_vtol y verifica que existan ' ...
               'los bloques ToWorkspace_* conectados.']);
    end

    [t_x, x] = extraer_t_y_valores(x_ws);
    [t_F, F] = extraer_t_y_valores(F_ws);
    [t_M, M] = extraer_t_y_valores(M_ws);

    % Extraer estados.
    pI = x(:,1:3);
    vI = x(:,4:6);
    q = x(:,7:10);
    w = x(:,11:13);

    % Figura 1: estados traslacionales.
    figure('Name', 'VTOL - Posicion y Velocidad', 'Color', 'w');
    tiledlayout(2,1);
    nexttile;
    plot(t_x, pI, 'LineWidth', 1.2);
    grid on; xlabel('t [s]'); ylabel('p_I [m]');
    legend('x','y','z', 'Location', 'best');
    title('Posicion inercial');
    nexttile;
    plot(t_x, vI, 'LineWidth', 1.2);
    grid on; xlabel('t [s]'); ylabel('v_I [m/s]');
    legend('v_x','v_y','v_z', 'Location', 'best');
    title('Velocidad inercial');

    % Figura 2: actitud y velocidades angulares.
    figure('Name', 'VTOL - Actitud y Velocidad Angular', 'Color', 'w');
    tiledlayout(2,1);
    nexttile;
    plot(t_x, q, 'LineWidth', 1.2);
    grid on; xlabel('t [s]'); ylabel('q');
    legend('q_0','q_1','q_2','q_3', 'Location', 'best');
    title('Cuaternion');
    nexttile;
    plot(t_x, w, 'LineWidth', 1.2);
    grid on; xlabel('t [s]'); ylabel('\omega_B [rad/s]');
    legend('\omega_x','\omega_y','\omega_z', 'Location', 'best');
    title('Velocidad angular en cuerpo');

    % Figura 3: fuerzas y momentos.
    figure('Name', 'VTOL - Fuerzas y Momentos', 'Color', 'w');
    tiledlayout(2,1);
    nexttile;
    plot(t_F, F, 'LineWidth', 1.2);
    grid on; xlabel('t [s]'); ylabel('F_B [N]');
    legend('F_x','F_y','F_z', 'Location', 'best');
    title('Fuerzas en marco cuerpo');
    nexttile;
    plot(t_M, M, 'LineWidth', 1.2);
    grid on; xlabel('t [s]'); ylabel('M_B [N m]');
    legend('M_x','M_y','M_z', 'Location', 'best');
    title('Momentos en marco cuerpo');
end

function ok = tiene_bloques_salida(nombre_modelo)
    req = {'ToWorkspace_estado_x', 'ToWorkspace_fuerza_body', 'ToWorkspace_momento_body'};
    ok = true;
    for i = 1:numel(req)
        b = find_system(nombre_modelo, 'SearchDepth', 1, 'Name', req{i});
        if isempty(b)
            ok = false;
            return;
        end
    end
end

function dato = obtener_variable_salida(simOut, nombre_var)
    dato = [];

    % 1) Intentar desde SimulationOutput.
    try
        tmp = simOut.get(nombre_var);
        if ~isempty(tmp)
            dato = tmp;
            return;
        end
    catch
    end

    % 2) Intentar desde workspace base.
    try
        if evalin('base', sprintf('exist(''%s'',''var'')', nombre_var)) ~= 0
            dato = evalin('base', nombre_var);
            return;
        end
    catch
    end
end

function [t, valores] = extraer_t_y_valores(dato)
    % Soporta estructura "Structure With Time" y formatos de timeseries.
    if isstruct(dato) && isfield(dato, 'time') && isfield(dato, 'signals')
        t = dato.time;
        valores = dato.signals.values;
    elseif isa(dato, 'timeseries')
        t = dato.Time;
        valores = dato.Data;
    else
        error('Formato de salida no soportado para graficar.');
    end

    t = t(:); % vector columna
    valores = squeeze(valores);

    % Asegurar matriz 2D [N x M].
    if isvector(valores)
        valores = valores(:);
    end
    if size(valores,1) ~= numel(t) && size(valores,2) == numel(t)
        valores = valores.';
    end
    if size(valores,1) ~= numel(t)
        error('Dimension incompatible entre tiempo y datos de salida.');
    end
end
