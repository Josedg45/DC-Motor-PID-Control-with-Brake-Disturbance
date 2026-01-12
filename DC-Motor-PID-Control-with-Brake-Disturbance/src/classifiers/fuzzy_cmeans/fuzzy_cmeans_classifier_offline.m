%% ===============================================================
%   ALGORITMO FCM (FUZZY C-MEANS) AUTOCONTENIDO Y CONFIGURABLE
%   - Permite elegir número de grupos y tipo de inicialización
%   - Figura 2 corregida: muestra señales y clasificación final
% ===============================================================
clear; clc; close all;
%% --- CARGA DE DATOS ---
X = load("datos_iniciales_FCM.txt");
% table = readtable("datos_control_pololu.csv");
% table = removevars(table, 'iso_time');
% X = table2array(table);
%%
[num_muestras, num_vars] = size(X);
%% --- NORMALIZACIÓN ENTRE 0 Y 1 ---
minimos = min(X);
maximos = max(X);
for j = 1:num_vars
    X(:, j) = (X(:, j) - minimos(j)) / (maximos(j) - minimos(j));
end
%% --- INGRESO DEL NÚMERO DE GRUPOS ---
num_gru = inf;
while num_gru > num_muestras
    num_gru = input('Ingrese el número de grupos: ');
    if num_gru > num_muestras
        disp('El número de grupos no puede ser mayor al número de datos.');
    end
end
c = num_gru;   % alias para claridad
%% --- SELECCIÓN DE CENTROIDES INICIALES ---
opc = input('¿Cómo seleccionar centroides? Primeros (1) o Aleatorios (2): ');
switch opc
    case 1
        V = X(1:c, :);           % primeros centroides
    case 2
        pos = randperm(num_muestras, c);  % aleatorios
        V = X(pos, :);
    otherwise
        error('Opción no válida.');
end
%% --- PARÁMETROS FCM ---
m = 2;                 % parámetro de difusividad (m > 1)
itermax = 250;         % iteraciones máximas
tol = 1e-6;            % tolerancia de convergencia
iter = 0;              % contador de iteraciones
V_old = zeros(size(V));
%% --- MATRIZ DE PERTENENCIA ---
U = zeros(num_muestras, c);
%% --- BUCLE PRINCIPAL DE FCM ---
while norm(V - V_old) > tol && iter < itermax
    iter = iter + 1;
    V_old = V;
    % --- Calcular grados de pertenencia ---
    for i = 1:num_muestras
        for k = 1:c
            dist_ik = norm(X(i,:) - V(k,:))^2;
            if dist_ik == 0
                U(i,:) = 0;
                U(i,k) = 1;
            else
                denom = 0;
                for j = 1:c
                    dist_ij = norm(X(i,:) - V(j,:))^2;
                    denom = denom + (dist_ik / dist_ij)^(1/(m-1));
                end
                U(i,k) = 1 / denom;
            end
        end
    end
    % --- Actualizar centroides ---
    for k = 1:c
        um = U(:,k).^m;
        V(k,:) = (um' * X) / sum(um);
    end
end
fprintf('\nConvergencia alcanzada tras %d iteraciones.\n', iter);
%% --- DESNORMALIZAR DATOS ---
X_desnorm = zeros(size(X));
for j = 1:num_vars
    X_desnorm(:, j) = X(:, j) * (maximos(j) - minimos(j)) + minimos(j);
end
%% --- ASIGNACIÓN DE GRUPOS (MÁXIMA PERTENENCIA) ---
[~, grupos] = max(U, [], 2);
%% --- FIGURA 1: CLASIFICACIÓN Y GRADOS DE PERTENENCIA ---
figure(1);
subplot(2,1,1);
plot(1:num_muestras, grupos, 'o');
grid on;
title('Clasificación final de los datos (FCM)');
xlabel('Índice de muestra');
ylabel('Grupo asignado');
subplot(2,1,2);
plot(U);
grid on;
title('Grados de pertenencia (U)');
xlabel('Índice de muestra');
ylabel('Pertenencia');
%% --- FIGURA 2: SEÑALES Y CLASIFICACIÓN ---
figure(2);
subplot(2,1,1);
plot(X_desnorm);
grid on;
title('Señales de la base de datos (normalizadas)');
xlabel('Índice de muestra');
ylabel('Valor normalizado');
subplot(2,1,2);
plot(1:num_muestras, grupos, 'o');
grid on;
title('Clasificación final (por muestra)');
xlabel('Índice de muestra');
ylabel('Grupo asignado');
%% --- GUARDAR CENTROIDES ---
nombre_archivo = 'centroides_FCM.txt';   % o 'centroides_FCM.csv'
writematrix(V, nombre_archivo);
