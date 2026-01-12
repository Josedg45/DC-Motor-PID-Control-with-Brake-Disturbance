%% ==============================================================
%  ESP32 Real-Time Data Acquisition + Fixed-FCM Clustering
%  ------------------------------------------------------------
%  Usa centroides cargados para FCM (fuzzy c-means) y calcula grados
%  de pertenencia en tiempo real (m = 2). Asigna cluster por máxima
%  pertenencia para visualización.
%  --------------------------------------------------------------
clear; close all; clc;
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

%% === SERIAL SETUP ===
port = "COM6";      % ⚠️ Cambia si hace falta
baud = 115200;

esp = serialport(port, baud);
configureTerminator(esp, "LF");
flush(esp);

%% === PARAMETERS ===
maxSamples = 200000;     % safety limit
num_gru = 6;             % número de clusters (se puede sobrescribir por los centroides cargados)
Ts = 0.01;               % 10 ms sampling
plotInterval = 0.25;      % plot update interval
windowSeconds = 60;     % ventana fija

data = zeros(maxSamples,3);
t = zeros(maxSamples,1);
i = 0;

% Fixed normalization limits (ajusta si tus variables tienen otros límites)
minimos = [0 0 0];
maximos = [100 100 100];

disp("Listening to ESP32... Press Ctrl+C to stop.");

%% === CARGAR CENTROIDES FCM ===
% Intenta cargar 'centroides_FCM.mat' (variable V) o 'centroides_FCM.txt'/'csv'.
V = []; % centroides por defecto (se llenará con los cargados)
if isfile('centroides_FCM.mat')
    S = load('centroides_FCM.mat');
    if isfield(S,'V')
        V = S.V;
        disp('Centroides cargados desde centroides_FCM.mat');
    end
elseif isfile('centroides_FCM.txt') || isfile('centroides_FCM.csv')
    if isfile('centroides_FCM.txt')
        M = readmatrix('centroides_FCM.txt');
        disp('Centroides cargados desde centroides_FCM.txt');
    else
        M = readmatrix('centroides_FCM.csv');
        disp('Centroides cargados desde centroides_FCM.csv');
    end
    % Normalmente en tu FCM anterior V era (c x num_vars). Aseguramos orientación:
    % Queremos v como (num_vars x c) para trabajar con z (num_vars x N).
    [r,cM] = size(M);
    if r > cM  % probablemente cada fila = un centroide con variables en columnas -> c x n
        % Transponer si hace falta: ahora M es (c x n) -> queremos (n x c)
        V = M';
    else
        % si M ya viene como (n x c) o (c x n), tratamos:
        if r == 3  % asumiendo 3 variables -> (n x c)
            V = M;
        else
            V = M'; % fallback: transponer
        end
    end
end

% Si no se encuentran centroides, caerá a clustering dinámico (opcional)
if isempty(V)
    warning('No se encontraron centroides cargados. Se usará K-means inicial (aleatorio) como fallback.');
    % Para fallback definimos V aleatorio (usa valores de z más adelante al primer plot)
    V = []; % lo inicializaremos dentro de la función la primera vez que haya datos
else
    % Ajustar num_gru según centroides cargados
    num_gru = size(V, 2);
    fprintf('Número de centroides cargados: %d\n', num_gru);
end

%% === SETUP PLOTS ===
figure('Name','ESP32 Real-Time Monitoring - FCM fixed centroids','NumberTitle','off');

subplot(2,1,1);
hold on; grid on;
ref_plot = plot(NaN,NaN,'r','LineWidth',1.2,'DisplayName','Setpoint [RPM]');
u_plot   = plot(NaN,NaN,'g','LineWidth',1.2,'DisplayName','Ley de control [\%]');
rpm_plot = plot(NaN,NaN,'b','LineWidth',1.2,'DisplayName','Velocidad [RPM]');
xlabel('Time [s]');
ylabel('Value');
title('Real-Time Signals');
legend('show','Location','southeast');
xlim([0 windowSeconds]);
ylim([0 110]);

subplot(2,1,2);
hold on; grid on;
cluster_plot = plot(NaN,NaN,'mo','MarkerSize',4,'DisplayName','Cluster');
xlabel('Time [s]');
ylabel('Cluster');
title('Real-Time FCM (max pertenencia)');
xlim([0 windowSeconds]);
ylim([0 num_gru+1]);
drawnow;

%% === ACQUISITION LOOP ===
tic;
lastPlotTime = 0;
sent70 = false;
sent100 = false;

% Diagnóstico: imprimir estado cada N segundos
diagInterval = 5; diagLast = 0;

% Opcional: no calcular FCM en cada frame si hay muchas muestras
fcmEvery = 1; % =1 cada vez; >1 cada n frames por cálculo
fcmCounter = 0;

while toc < windowSeconds  % o while true para indefinido
    
    % === SETPOINT SCHEDULER ===
    currentTime = toc;
    if currentTime >= 5 && ~sent70
        writeline(esp,"S70");
        disp("➡️  Setpoint cambiado a 70 RPM (t = 5s)");
        sent70 = true;
    elseif currentTime >= 20 && ~sent100
        writeline(esp,"S100");
        disp("➡️  Setpoint cambiado a 100 RPM (t = 20s)");
        sent100 = true;
    end

    % === LECTURA SERIAL (con try/catch) ===
    try
        if esp.NumBytesAvailable > 0
            line = readline(esp);            % puede lanzar error si terminador distinto
            vals = str2double(strsplit(strtrim(line), ','));
            if numel(vals) == 3 && all(~isnan(vals))
                i = i + 1;
                if i > maxSamples, break; end
                vals(2) = (vals(2)/255)*100;
                data(i,:) = vals;
                t(i) = (i-1) * Ts;
            end
        end
    catch ME
        warning('Error leyendo serial: %s', ME.message);
        % Intentamos vaciar buffer para recuperarnos
        if esp.NumBytesAvailable > 0
            try
                read(esp, esp.NumBytesAvailable, "char"); % vaciar
                warning('Buffer serial vaciado tras excepción.');
            catch
                warning('No se pudo vaciar buffer.');
            end
        end
    end

    % Si el buffer crece mucho, lo vaciamos (posible causa de estancamiento)
    if esp.NumBytesAvailable > 5000
        warning('Serial buffer muy grande: %d bytes. Vaciando buffer...', esp.NumBytesAvailable);
        read(esp, esp.NumBytesAvailable, "char");
    end

    % === ACTUALIZAR GRÁFICAS (cada plotInterval) ===
    if toc - lastPlotTime >= plotInterval && i > 10
        lastPlotTime = toc;
        fcmCounter = fcmCounter + 1;

        ztemp = data(1:i,:)';
        % normalizar
        for k = 1:3
            ztemp(k,:) = (ztemp(k,:) - minimos(k)) / (maximos(k) - minimos(k) + eps);
        end

        % Ejecutar FCM de forma menos frecuente si fcmEvery>1
        if mod(fcmCounter, fcmEvery) == 0
            grupos = runFCM_fixedCentroids(ztemp, V, 2); % m = 2
        end

        visibleIdx = t(1:i) <= windowSeconds;
        visibleN = sum(visibleIdx);

        % Recortar vectores a la misma longitud para evitar errores
        set(ref_plot,'XData',t(visibleIdx),'YData',data(visibleIdx,1));
        set(u_plot,'XData',t(visibleIdx),'YData',data(visibleIdx,2));
        set(rpm_plot,'XData',t(visibleIdx),'YData',data(visibleIdx,3));
        if exist('grupos','var') && numel(grupos) >= visibleN
            set(cluster_plot,'XData',t(visibleIdx),'YData',grupos(1:visibleN));
        else
            % si no hay suficientes etiquetas, pintamos NaN para que no falle
            set(cluster_plot,'XData',t(visibleIdx),'YData',nan(1,visibleN));
        end

        drawnow; % forzar actualización (diagnóstico). vuelve a 'limitrate' si quieres
    end

    % === DIAGNÓSTICO PERIÓDICO ===
    if toc - diagLast >= diagInterval
        diagLast = toc;
        fprintf('[diag] t=%.1fs, muestras=%d, bytesAvail=%d\n', toc, i, esp.NumBytesAvailable);
    end

end
%% === FINAL CLUSTERING (usando centroides cargados si existen) ===
z = data';
for k = 1:3
    z(k,:) = (z(k,:) - minimos(k)) / (maximos(k) - minimos(k) + eps);
end

grupos = runFCM_fixedCentroids(z, V, 2);

figure;
subplot(2,1,1);
hold on;
plot(t(1:i), data(1:i,1), 'r', 'LineWidth', 1.5);
plot(t(1:i), data(1:i,2), 'g', 'LineWidth', 1.5);
plot(t(1:i), data(1:i,3), 'b', 'LineWidth', 1.5);
legend('Setpoint [RPM]','Ley de control [\%]','Velocidad [RPM]','Location','southeast');
xlabel('Time [s]');
ylabel('Value');
title('Full Dataset');
grid on;

subplot(2,1,2);
Nplot = min(length(t(1:i)), length(grupos));
plot(t(1:Nplot), grupos(1:Nplot), 'ko');
xlabel('Time [s]');
ylabel('Cluster');
title('Final FCM clusters (max pertenencia)');
grid on;

disp('✅ Clustering complete.');

%% === FUNCIONES ===

function grupos = runFCM_fixedCentroids(z, Vloaded, m)
    % z: (n x N) datos normalizados
    % Vloaded: si no está vacío -> centroides (n x c). Si vacío -> inicializa centroids con kmeans
    % m: exponente de fuzziness (>=1, típicamente 2)
    [n, N] = size(z);

    % Si no hay centroides cargados, inicializamos con kmeans sobre z (fallback)
    if isempty(Vloaded)
        c = 6; % valor por defecto si no hay centroides guardados (puedes cambiar)
        % inicializamos centroids con KMeans para tener algo usable
        pos = randperm(N, min(c,N));
        V = z(:, pos);
    else
        V = Vloaded;
        % Aseguramos que V tenga forma (n x c)
        if size(V,1) ~= n && size(V,2) == n
            V = V'; % transponer si venía (c x n)
        end
    end
    c = size(V, 2);

    % Calculamos distancias (c x N), evitando 0 por numerics
    d = zeros(c, N);
    for i = 1:c
        dif = z - V(:, i);
        d(i, :) = sum(dif.^2, 1); % distancia al cuadrado (sin sqrt)
    end
    % Reemplazar ceros por un valor muy pequeño para evitar división por cero
    zero_mask = d == 0;
    if any(zero_mask, 'all')
        % Cuando distancia==0, la pertenencia será 1 para ese centroide y 0 para los demás
        % Lo manejamos más abajo directamente.
    end

    % Calcular U (c x N)
    U = zeros(c, N);
    for k = 1:N
        if any(d(:,k) == 0)
            idx0 = find(d(:,k) == 0, 1, 'first');
            U(:,k) = 0;
            U(idx0,k) = 1;
        else
            for i = 1:c
                denom = sum( (d(i,k) ./ d(:,k)).^(1/(m-1)) );
                U(i,k) = 1 / denom;
            end
        end
    end

    % Asignación hard por máxima pertenencia (para graficar)
    [~, idx_max] = max(U, [], 1);
    grupos = idx_max; % 1 x N

    % Devolver en formato fila (igual al uso en plots)
    grupos = reshape(grupos, [1, N]);
end
