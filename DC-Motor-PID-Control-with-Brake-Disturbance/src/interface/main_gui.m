%% ==============================================================
%  ESP32 Real-Time Data Acquisition + Hard K-Means Clustering
%  ------------------------------------------------------------
%  Sends setpoint changes at 5s and 30s automatically
%  --------------------------------------------------------------
clear; close all; clc;
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

%% === SERIAL SETUP ===
port = "COM6";      % ⚠️ Change as needed
baud = 115200;

esp = serialport(port, baud);
configureTerminator(esp, "LF");
flush(esp);

%% === PARAMETERS ===
maxSamples = 100000;     % safety limit
num_gru = 6;             % number of clusters
Ts = 0.01;               % 10 ms sampling
plotInterval = 0.1;      % plot update interval
windowSeconds = 120;      % fixed 60 s window

data = zeros(maxSamples,3);
t = zeros(maxSamples,1);
i = 0;

% Fixed normalization limits
minimos = [0 0 0];
maximos = [100 100 100];

disp("Listening to ESP32... Press Ctrl+C to stop.");

%% === SETUP PLOTS ===
figure('Name','ESP32 Real-Time Monitoring','NumberTitle','off');

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
title('Real-Time Clustering');
xlim([0 windowSeconds]);
ylim([0 num_gru+1]);
drawnow;

%% === ACQUISITION LOOP ===
tic;
lastPlotTime = 0;
sent70 = false;
sent100 = false;

while toc < windowSeconds  % or use `while true` for indefinite run
    
    % === SETPOINT SCHEDULER ===
    currentTime = toc;
    if currentTime >= 5 && ~sent70
        writeline(esp,"S70");
        disp("➡️  Setpoint cambiado a 70 RPM (t = 5s)");
        sent70 = true;
    elseif currentTime >= 30 && ~sent100
        writeline(esp,"S100");
        disp("➡️  Setpoint cambiado a 100 RPM (t = 30s)");
        sent100 = true;
    else
        % Lectura continua
        % writeline(esp,"S80"); % Comando base (puedes quitarlo si no es necesario)
    end

    % === LECTURA SERIAL ===
    if esp.NumBytesAvailable > 0
        line = readline(esp);
        vals = str2double(strsplit(strtrim(line), ','));
        if numel(vals) == 3 && all(~isnan(vals))
            i = i + 1;
            if i > maxSamples, break; end
            vals(2) = (vals(2)/255)*100;
            data(i,:) = vals;
            t(i) = (i-1) * Ts;
        end
    end

    % === ACTUALIZAR GRÁFICAS ===
    if toc - lastPlotTime >= plotInterval && i > 10
        lastPlotTime = toc;

        ztemp = data(1:i,:)';
        for k = 1:3
            ztemp(k,:) = (ztemp(k,:) - minimos(k)) / (maximos(k) - minimos(k) + eps);
        end
        grupos = runKMeans(ztemp, num_gru);

        visibleIdx = t(1:i) <= windowSeconds;
        set(ref_plot,'XData',t(visibleIdx),'YData',data(visibleIdx,1));
        set(u_plot,'XData',t(visibleIdx),'YData',data(visibleIdx,2));
        set(rpm_plot,'XData',t(visibleIdx),'YData',data(visibleIdx,3));
        set(cluster_plot,'XData',t(visibleIdx),'YData',grupos(visibleIdx));

        drawnow limitrate;
    end
end

data = data(1:i,:);
disp("✅ Data acquisition finished.");

%% === FINAL CLUSTERING ===
z = data';
for k = 1:3
    z(k,:) = (z(k,:) - minimos(k)) / (maximos(k) - minimos(k) + eps);
end
grupos = runKMeans(z, num_gru);

figure;
subplot(2,1,1);
hold on;
plot(t(1:i), data(1:i,1), 'r', 'LineWidth', 1.5);   % Setpoint [RPM] → azul
plot(t(1:i), data(1:i,2), 'g', 'LineWidth', 1.5);   % Ley de control [%] → verde
plot(t(1:i), data(1:i,3), 'b', 'LineWidth', 1.5);   % Velocidad [RPM] → rojo
legend('Setpoint [RPM]','Ley de control [\%]','Velocidad [RPM]','Location','southeast');
xlabel('Time [s]');
ylabel('Value');
title('Full Dataset');
grid on;

subplot(2,1,2);
plot(t(1:i), grupos, 'm');
xlabel('Time [s]');
ylabel('Cluster');
title('Final Clusters');
grid on;

disp('✅ Clustering complete.');

%% === FUNCTIONS ===
function grupos = runKMeans(z, num_gru)
    [n, N] = size(z);
    rng('shuffle');
    pos = randperm(N, num_gru);
    v = z(:, pos);
    u_prev = ones(num_gru, N);
    ite = 0; cambio = true;
    while cambio && ite < 100
        d = distancia(num_gru, N, n, z, v);
        u = agr_min_dis(d);
        if isequal(u, u_prev)
            cambio = false;
        else
            u_prev = u;
            v = calculo_centroides(z, u);
            ite = ite + 1;
        end
    end
    grupos = zeros(1, N);
    for j = 1:num_gru
        grupos(u(j, :) == 1) = j;
    end
end

function d = distancia(num_gru, N, n, z, v)
    d = zeros(num_gru, N);
    for i = 1:num_gru
        dif = z - v(:, i);
        d(i, :) = sqrt(sum(dif.^2, 1));
    end
end

function u = agr_min_dis(d)
    [~, idx] = min(d, [], 1);
    [num_gru, N] = size(d);
    u = zeros(num_gru, N);
    for k = 1:N
        u(idx(k), k) = 1;
    end
end

function v = calculo_centroides(z, u)
    [n, ~] = size(z);
    num_gru = size(u, 1);
    v = zeros(n, num_gru);
    for i = 1:num_gru
        miembros = z(:, u(i, :) == 1);
        if ~isempty(miembros)
            v(:, i) = mean(miembros, 2);
        else
            v(:, i) = z(:, randi(size(z,2)));
        end
    end
end
%%
writematrix(data,'datos_iniciales_FCM');