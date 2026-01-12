%% ALGORITMO C-MEANS (K-MEANS DURO) - SCRIPT AUTOCONTENIDO CORREGIDO
clear; close all; clc;
%% --- CARGA DE DATOS ---
z = load('datos_iniciales_FCM.txt');   % Cargar base de datos
[fil, col] = size(z);
%% --- ESCALADO DE DATOS ENTRE 0 Y 1 ---
minimos = min(z);
maximos = max(z);
zres = zeros(fil, col);
for i = 1:col
    for j = 1:fil
        zres(j, i) = (z(j, i) - minimos(i)) / (maximos(i) - minimos(i));
    end
end
z = zres';           % cada columna es un dato
[n, N] = size(z);    % n = # variables, N = # datos
%% --- INGRESO DEL NÚMERO DE GRUPOS ---
num_gru = inf;
while num_gru > N
    num_gru = input('Ingrese el número de grupos: ');
    if num_gru > N
        disp('El número de grupos no puede ser mayor al número de datos.');
    end
end
%% --- SELECCIÓN DE CENTROIDES INICIALES ---
opc = input('¿Cómo seleccionar centroides? Primeros (1) o Aleatorios (2): ');
switch opc
    case 1
        v = z(:, 1:num_gru);           % primeros centroides
    case 2
        pos = randperm(N, num_gru);   % selección aleatoria sin repetición
        v = z(:, pos);
    otherwise
        error('Opción no válida.');
end
%% --- INICIALIZACIÓN ---
u_prev = ones(num_gru, N);   % matriz de pertenencia inicial
ite = 0;
cambio = true;
%% --- ITERACIÓN PRINCIPAL ---
while cambio
    % 1. Calcular distancias
    d = distancia(num_gru, N, n, z, v);
    % 2. Asignar pertenencia según mínima distancia (K-means duro)
    u = agr_min_dis(d);
    % 3. Verificar convergencia y actualizar centroides si procede
    if isequal(u, u_prev)
        cambio = false;
    else
        u_prev = u;
        v = calculo_centroides(z, u);
        ite = ite + 1;
    end
end
fprintf('\nConvergencia alcanzada tras %d iteraciones.\n', ite);
%% --- AGRUPAR RESULTADOS PARA VISUALIZACIÓN ---
grupos = zeros(1, N);
for j = 1:num_gru
    grupos(u(j, :) == 1) = j;
end
figure(1);
subplot(2, 1, 1);
plot(1:N, grupos, 'o'); grid on;
title('Clasificación final de los datos (por muestra)');
xlabel('Índice de muestra');
ylabel('Grupo asignado');
subplot(2, 1, 2);
plot(z'); grid on;
title('Datos normalizados (cada fila = variable)');
xlabel('Índice de muestra');
ylabel('Valor normalizado');
%% --- CLASIFICACIÓN / VALIDACIÓN (USANDO CENTROIDES EN LA MISMA ESCALA) ---
% dis_centro ahora calcula distancias en la misma escala normalizada que z
uval = dis_centro(v, z);
% construir vector de grupos por muestra para validación
grupos_val = zeros(1, N);
for j = 1:num_gru
    grupos_val(uval(j, :) == 1) = j;
end
figure(2);
plot(1:N, grupos_val, 'o'); grid on;
title('Validación: grupo asignado por muestra');
xlabel('Índice de muestra');
ylabel('Grupo asignado');
%% ===============================================================
% FUNCIONES INTERNAS
%% ===============================================================
function d = distancia(num_gru, N, n, z, v)
    % Calcula la distancia euclidiana entre cada dato (columnas de z)
    % y cada centroide (columnas de v). z y v deben estar en la misma escala.
    d = zeros(num_gru, N);
    for i = 1:num_gru
        dif = z - v(:, i);
        d(i, :) = sqrt(sum(dif.^2, 1));
    end
end
function u = agr_min_dis(d)
    % Asigna cada punto al grupo con la distancia mínima
    [~, idx] = min(d, [], 1);        % idx: índice del centroide más cercano por columna
    [num_gru, N] = size(d);
    u = zeros(num_gru, N);
    for k = 1:N
        u(idx(k), k) = 1;
    end
end
function v = calculo_centroides(z, u)
    % Calcula los nuevos centroides como la media de los puntos asignados
    [n, ~] = size(z);
    num_gru = size(u, 1);
    v = zeros(n, num_gru);
    for i = 1:num_gru
        miembros = z(:, u(i, :) == 1);
        if ~isempty(miembros)
            v(:, i) = mean(miembros, 2);
        else
            % Si un cluster quedó vacío, mantener el centroide previo (no hay previo aquí)
            % mejor: reinicializarlo con un punto aleatorio
            rng('shuffle');
            pos = randi(size(z, 2));
            v(:, i) = z(:, pos);
        end
    end
end
function uval = dis_centro(v, z)
    % Clasifica los puntos según su centroide más cercano (validación)
    % IMPORTANTE: v y z deben estar en la misma escala (ambos normalizados)
    num_gru = size(v, 2);
    N = size(z, 2);
    d = zeros(num_gru, N);
    for i = 1:num_gru
        dif = z - v(:, i);
        d(i, :) = sqrt(sum(dif.^2, 1));
    end
    [~, idx] = min(d, [], 1);
    uval = zeros(num_gru, N);
    for k = 1:N
        uval(idx(k), k) = 1;
    end
end
%%
% writematrix(v,"centroides_KM.txt")