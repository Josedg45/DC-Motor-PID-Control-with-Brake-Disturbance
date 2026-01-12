function interfaz_setpoint_botones
    % === Crear la figura ===
    fig = uifigure('Name', 'Control de Setpoint', 'Position', [500 300 300 250]);

    % === Crear conexión serial ===
    puerto = 'COM6'; % Cambia esto según tu puerto
    baud = 115200;
    s = serialport(puerto, baud);
    configureTerminator(s, "LF");
    
    % === Etiqueta de estado ===
    lbl = uilabel(fig, 'Text', 'Setpoint actual: -', ...
        'FontSize', 14, 'Position', [60 190 200 40]);

    % === Botón Setpoint Bajo ===
    btn_bajo = uibutton(fig, 'push', ...
        'Text', 'Setpoint Bajo (50)', ...
        'Position', [80 130 140 40], ...
        'ButtonPushedFcn', @(btn,event) enviarSetpoint(50, s, lbl));

    % === Botón Setpoint Medio ===
    btn_medio = uibutton(fig, 'push', ...
        'Text', 'Setpoint Medio (100)', ...
        'Position', [80 80 140 40], ...
        'ButtonPushedFcn', @(btn,event) enviarSetpoint(100, s, lbl));

    % === Botón Setpoint Alto ===
    btn_alto = uibutton(fig, 'push', ...
        'Text', 'Setpoint Alto (150)', ...
        'Position', [80 30 140 40], ...
        'ButtonPushedFcn', @(btn,event) enviarSetpoint(150, s, lbl));

    % === Cierre seguro ===
    fig.CloseRequestFcn = @(src, event) cerrarPuerto(src, s);
end

function enviarSetpoint(valor, s, lbl)
    writeline(s, num2str(valor));   % Enviar setpoint por serial
    lbl.Text = ['Setpoint actual: ', num2str(valor)];
end

function cerrarPuerto(fig, s)
    if isvalid(s)
        clear s
    end
    delete(fig);
end
