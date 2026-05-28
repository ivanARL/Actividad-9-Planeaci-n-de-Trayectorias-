%% Differential Drive - Trayectoria Continua Global (Sin Teletransportación)
clear classes;
clear functions;
close all;
clc;

%% 1. Configuración del Vehículo
R = 0.1;                        % Radio de la rueda [m]
L = 0.5;                        % Distancia entre ruedas [m]
dd = DifferentialDrive(R,L);
sampleTime = 0.05;              % Tiempo de muestreo [s]

%% 2. Matriz Unificada de Waypoints (Flujo Continuo)
waypointsGlobales = [
    % Ojo Izquierdo
    -3.14, 1.74; -2.5, 2; -1.71, 1.79; -1.5, 1.22; ...
    -1.52, 0.89; -1.48, 0.57; -1.75, 0.56; -1.93, 0.8; ...
    -2.42, 0.89; -2.95, 1.29; ...
    
    % Ojo Derecho
     3.65, 1.79; 2.93, 2.03; 2.3, 1.95; 1.86, 1.58; ...
     1.77, 1.16; 1.78, 0.6; 2.13, 0.59; ...
     2.16, 0.89; 2.69, 0.94; 3.25, 1.27; ...
     
    % Hocico
    -1.39, 0.58; -1.18, 0.11; -1.45, -0.58; ...
    -1.68, -1.12; -1.63, -1.62; -1.15, -2.42; ...
    -0.98, -2.17; -0.52, -2.27; -0.01, -2.35; ...
     0.5, -2.29; 1.02, -2.18; 1.27, -2.43; 1.68, -1.64;...
     1.75, -1.05; 1.5, -0.5; 1.32, 0.12; 1.56, 0.66;...
     
    % Nariz
    -0.99, -2.55; -0.69, -2.59; -0.45, -2.76; -0.29, -3.11;...
    -0.22, -3.43; -0.08, -3.65; 0.14, -3.69; 0.32, -3.52; ...
     0.42, -3.14; 0.54, -2.84; 0.8, -2.6; 1.14, -2.48;...
     
    % Boca
    -1.69, -4.99; -1.09, -4.85; -0.59, -4.63; -0.07, -4.39;...
     0.1, -3.97; 0.19, -4.39; 0.59, -4.53; 1.09, -4.74; 1.6, -4.8;... 
     
    % Cara
    -4.93, 0.19; -4.7, -0.68; -4.35, -1.39; -3.68, -2; -2.85, -2.42;...
    -2.71, -3.31; -2.48, -3.92; -2.18, -4.71; -1.44, -5.3; -0.53, -5.75;...
     0.52, -5.71; 1.55, -5.26; 2.31, -4.57; 2.74, -3.74; 3, -3;...
     2.95, -2.15; 3.85, -1.76; 4.68, -1.47; ...
     
    % Cuerpo
    -8.01, -1.46; -6.61, -1.17; -5.7, -0.33; -4.97, 0.73; -5.12, 1.87;...
    -5.23, 2.7; -5.74, 3.35; -5.35, 4.15; -5.97, 4.33; -6.29, 5.02;...
    -6.52, 5.79; -6.35, 6.62; -5.82, 7.18; -4.98, 7.22; -4.35, 6.85;...
    -3.73, 6.43; -3.17, 5.96; -2.32, 6.17; -1.48, 6.63; -0.41, 6.88;...
     0.79, 6.9; 1.77, 6.73; 2.87, 6.2; 3.32, 5.99; 4.06, 6.66;...
     4.97, 7.18; 5.87, 7.4; 6.6, 7.11; 7.01, 6.33; 6.88, 5.3;...
     6.55, 4.49; 5.84, 4.26; 6.28, 3.49; 5.81, 2.77; 5.61, 1.99;...
     5.61, 1.01; 5.46, -0.03; 5.17, -0.84; 4.51, -1.76; 5.24, -3.02;...
     5.61, -4.14; 5.85, -5.33; 5.94, -6.56; 6, -8 
];

%% 3. Inicialización de Posición, Controlador y Visualizador
% El robot se coloca UNA SOLA VEZ al inicio de toda la ruta
theta_init = atan2(waypointsGlobales(2,2) - waypointsGlobales(1,2), ...
                   waypointsGlobales(2,1) - waypointsGlobales(1,1));
curPose = [waypointsGlobales(1,1); waypointsGlobales(1,2); theta_init];

viz = Visualizer2D;
viz.hasWaypoints = true;

% Configuración del único bloque Pure Pursuit
controller = controllerPurePursuit;
controller.Waypoints = waypointsGlobales;       
controller.LookaheadDistance = 0.25;   % Sintonización para seguir curvas cerradas
controller.DesiredLinearVelocity = 1.2; 
controller.MaxAngularVelocity = 6.0;   

r = rateControl(1/sampleTime);
fprintf('Iniciando el seguimiento lineal continuo de todos los puntos...\n');

%% 4. Bucle único de simulación
while true
    % Calcular las velocidades requeridas basadas en la pose actual
    [vRef, wRef] = controller(curPose);
    
    % Aplicar cinemática diferencial del robot
    velB = [vRef; 0; wRef];
    vel = bodyToWorld(velB, curPose);
    curPose = curPose + vel * sampleTime;
    
    % Actualizar la animación gráfica
    viz(curPose, waypointsGlobales);
    xlim([-10 10]);
    ylim([-10 10]);
    
    % Condición de parada: llegar al último punto del cuerpo (6, -8)
    distAlFinal = norm(curPose(1:2) - waypointsGlobales(end,:)');
    if distAlFinal < 0.25
        break; 
    end
    
    waitfor(r);
end

fprintf('\n¡Trayectoria lineal completada de corrido y sin interrupciones!\n');