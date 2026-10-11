function dashboard_vtol(nombre_modelo)
% DASHBOARD_VTOL Panel interactivo para probar entradas del VTOL.
% Permite ajustar las 9 entradas con sliders y ejecutar simulacion + graficas.
%
% Entradas del vector u:
% [empuje; epsilon; gamma_izq; gamma_der; alfa_rud_izq; gamma_rud_izq; alfa_rud_der; gamma_rud_der; beta_aileron]

    if nargin < 1 || isempty(nombre_modelo)
        nombre_modelo = 'planta_vtol';
    end

    p = crear_parametros_vtol();
    u0 = [p.m * p.g; 0; 0; 0; 0; 0; 0; 0; 0];
    limites = [
        0,          2.2 * p.m * p.g;   % empuje [N]
       -deg2rad(45), deg2rad(45);      % epsilon [rad]
       -deg2rad(35), deg2rad(35);      % gamma_izq [rad]
       -deg2rad(35), deg2rad(35);      % gamma_der [rad]
       -deg2rad(30), deg2rad(30);      % alfa_rud_izq [rad]
       -deg2rad(30), deg2rad(30);      % gamma_rud_izq [rad]
       -deg2rad(30), deg2rad(30);      % alfa_rud_der [rad]
       -deg2rad(30), deg2rad(30);      % gamma_rud_der [rad]
       -deg2rad(25), deg2rad(25)       % beta_aileron [rad]
    ];

    nombres = {
        'empuje [N]'
        'epsilon [rad]'
        'gamma_izq [rad]'
        'gamma_der [rad]'
        'alfa_rud_izq [rad]'
        'gamma_rud_izq [rad]'
        'alfa_rud_der [rad]'
        'gamma_rud_der [rad]'
        'beta_aileron [rad]'
    };

    fig = uifigure('Name', 'Dashboard VTOL', 'Position', [80 80 700 700]);
    gl = uigridlayout(fig, [14, 3]);
    gl.RowHeight = [{28}, repmat({32}, 1, 10), {36, 36, '1x'}];
    gl.ColumnWidth = {180, '1x', 120};
    gl.Padding = [10 10 10 10];

    titulo = uilabel(gl, 'Text', 'Dashboard de entradas - Planta VTOL', ...
        'FontWeight', 'bold', 'FontSize', 14);
    titulo.Layout.Row = 1;
    titulo.Layout.Column = [1 3];

    sliders = gobjects(9,1);
    valores = gobjects(9,1);
    for i = 1:9
        lbl = uilabel(gl, 'Text', nombres{i});
        lbl.Layout.Row = i + 1;
        lbl.Layout.Column = 1;

        sliders(i) = uislider(gl, ...
            'Limits', limites(i,:), ...
            'Value', u0(i), ...
            'ValueChangedFcn', @(src,~)actualizar_valor(i, src.Value), ...
            'ValueChangingFcn', @(src,evt)actualizar_valor(i, evt.Value));
        sliders(i).Layout.Row = i + 1;
        sliders(i).Layout.Column = 2;

        valores(i) = uilabel(gl, 'Text', num2str(u0(i), '%.4f'));
        valores(i).Layout.Row = i + 1;
        valores(i).Layout.Column = 3;
    end

    edt_tiempo = uieditfield(gl, 'numeric', ...
        'Value', 20, 'Limits', [1 300], 'RoundFractionalValues', false);
    edt_tiempo.Layout.Row = 12;
    edt_tiempo.Layout.Column = 2;
    lbl_tiempo = uilabel(gl, 'Text', 'tiempo_final [s]');
    lbl_tiempo.Layout.Row = 12;
    lbl_tiempo.Layout.Column = 1;

    btn_hover = uibutton(gl, 'Text', 'Preset hover', ...
        'ButtonPushedFcn', @(~,~)cargar_hover());
    btn_hover.Layout.Row = 13;
    btn_hover.Layout.Column = 1;

    btn_cero = uibutton(gl, 'Text', 'Preset cero', ...
        'ButtonPushedFcn', @(~,~)cargar_cero());
    btn_cero.Layout.Row = 13;
    btn_cero.Layout.Column = 2;

    btn_sim = uibutton(gl, 'Text', 'Simular y graficar', ...
        'ButtonPushedFcn', @(~,~)simular());
    btn_sim.Layout.Row = 13;
    btn_sim.Layout.Column = 3;

    txt_estado = uitextarea(gl, ...
        'Value', {'Listo. Ajusta sliders y presiona "Simular y graficar".'}, ...
        'Editable', 'off');
    txt_estado.Layout.Row = 14;
    txt_estado.Layout.Column = [1 3];

    function actualizar_valor(idx, val)
        valores(idx).Text = num2str(val, '%.4f');
    end

    function u = leer_u()
        u = zeros(9,1);
        for k = 1:9
            u(k) = sliders(k).Value;
        end
    end

    function escribir_u(u)
        for k = 1:9
            sliders(k).Value = u(k);
            valores(k).Text = num2str(u(k), '%.4f');
        end
    end

    function cargar_hover()
        u = [p.m * p.g; 0; 0; 0; 0; 0; 0; 0; 0];
        escribir_u(u);
        txt_estado.Value = {'Preset hover cargado.'};
    end

    function cargar_cero()
        escribir_u(zeros(9,1));
        txt_estado.Value = {'Preset cero cargado.'};
    end

    function simular()
        try
            u = leer_u();
            tfin = edt_tiempo.Value;
            txt_estado.Value = {'Corriendo simulacion...'};
            drawnow;
            simular_y_graficar_vtol(nombre_modelo, u, tfin);
            txt_estado.Value = {
                'Simulacion completada.'
                sprintf('Modelo: %s', nombre_modelo)
                sprintf('Tiempo final: %.2f s', tfin)
            };
        catch ME
            txt_estado.Value = {
                'Error al simular:'
                ME.message
            };
        end
    end
end
