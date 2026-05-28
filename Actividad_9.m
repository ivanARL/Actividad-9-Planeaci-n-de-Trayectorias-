%% Differential Drive - Dibujo por Capas (Sin líneas de conexión)
clear classes;
clear functions;
close all;
clc;

%% 1. Configuración del Vehículo
R = 0.1;                        % Radio de la rueda [m]
L = 0.5;                        % Distancia entre ruedas [m]
dd = DifferentialDrive(R,L);
sampleTime = 0.05;              % Tiempo de muestreo [s]

%% 2. Definición de Segmentos del Rostro
ojo_izq = [ -3.14, 1.74; -2.5, 2; -1.71, 1.79; -1.5, 1.22; ...
            -1.52, 0.89; -1.48, 0.57; -1.75, 0.56; -1.93, 0.8; ...
            -2.42, 0.89; -2.95, 1.29 ];

ojo_der = [  3.65, 1.79; 2.93, 2.03; 2.3, 1.95; 1.86, 1.58; ...
             1.77, 1.16; 1.78, 0.6; 2.13, 0.59; ...
             2.16, 0.89; 2.69, 0.94; 3.25, 1.27 ];

hocico  = [ -1.39, 0.58; -1.18, 0.11; -1.45, -0.58; ...
            -1.68, -1.12; -1.63, -1.62; -1.15, -2.42; ...
            -0.98, -2.17; -0.52, -2.27; -0.01, -2.35; ...
             0.5, -2.29; 1.02, -2.18; 1.27, -2.43; 1.68, -1.64;...
             1.75, -1.05; 1.5, -0.5; 1.32, 0.12; 1.56, 0.66 ];

nariz   = [ -0.99, -2.55; -0.69, -2.59; -0.45, -2.76; -0.29, -3.11;...
            -0.22, -3.43; -0.08, -3.65; 0.14, -3.69; 0.32, -3.52; ...
             0.42, -3.14; 0.54, -2.84; 0.8, -2.6; 1.14, -2.48 ];

boca    = [ -1.69, -4.99; -1.09, -4.85; -0.59, -4.63; -0.07, -4.39;...
             0.1, -3.97; 0.19, -4.39; 0.59, -4.53; 1.09, -4.74; 1.6, -4.8 ];

cara    = [ -4.93, 0.19; -4.7, -0.68; -4.35, -1.39; -3.68, -2; -2.85, -2.42;...
            -2.71, -3.31; -2.48, -3.92; -2.18, -4.71; -1.44, -5.3; -0.53, -5.75;...
             0.52, -5.71; 1.55, -5.26; 2.31, -4.57; 2.74, -3.74; 3, -3;...
             2.95, -2.15; 3.85, -1.76; 4.68, -1.47 ];

cuerpo  = [ -8.01, -1.46; -6.61, -1.17; -5.7, -0.33; -4.97, 0.73; -5.12, 1.87;...
            -5.23, 2.7; -5.74, 3.35; -5.35, 4.15; -5.97, 4.33; -6.29, 5.02;...
            -6.52, 5.79; -6.35, 6.62; -5.82, 7.18; -4.98, 7.22; -4.35, 6.85;...
            -3.73, 6.43; -3.17, 5.96; -2.32, 6.17; -1.48, 6.63; -0.41, 6.88;...
             0.79, 6.9; 1.77, 6.73; 2.87, 6.2; 3.32, 5.99; 4.06, 6.66;...
             4.97, 7.18; 5.87, 7.4; 6.6, 7.11; 7.01, 6.33; 6.88, 5.3;...
             6.55, 4.49; 5.84, 4.26; 6.28, 3.49; 5.81, 2.77; 5.61, 1.99;...
             5.61, 1.01; 5.46, -0.03; 5.17, -0.84; 4.51, -1.76; 5.24, -3.02;...
             5.61, -4.14; 5.85, -5.33; 5.94, -6.56; 6, -8 ];

% Almacenar los trazos en una celda para el bucle secuencial
segmentos = {ojo_izq, ojo_der, hocico, nariz, boca, cara, cuerpo};
nombresFacciones = {'Ojo Izquierdo', 'Ojo Derecho', 'Hocico', 'Nariz', 'Boca', 'Cara', 'Cuerpo'};

% Unimos todo solo para mostrar la guía completa de puntos rojos en el fondo
waypointsGlobales = cat(1, segmentos{:});

%% 3. Inicialización del Controlador y Visualizador
viz = Visualizer2D;
viz.hasWaypoints = true;

controller = controllerPurePursuit;
controller.LookaheadDistance = 0.22;   % Distancia corta para máxima precisión geométrica
controller.DesiredLinearVelocity = 1.2; 
controller.MaxAngularVelocity = 6.0;   

r = rateControl(1/sampleTime);

%% 4. Ejecución del Trazado por Facciones
for s = 1:numel(segmentos)
    fprintf('Dibujando facción: %s...\n', nombresFacciones{s});
    
    puntosActuales = segmentos{s};
    controller.Waypoints = puntosActuales;
    
    % Calcular la orientación inicial del robot apuntando al segundo punto del trazo
    theta_init = atan2(puntosActuales(2,2) - puntosActuales(1,2), ...
                       puntosActuales(2,1) - puntosActuales(1,1));
                   
    % Teletransportar físicamente al robot al inicio del nuevo trazo
    curPose = [puntosActuales(1,1); puntosActuales(1,2); theta_init];
    
    % --- EL TRUCO ---
    % Si no es la primera facción, enviamos un vector de NaNs al visualizador.
    % Esto le dice a MATLAB que rompa la línea de trayectoria actual aquí.
    if s > 1
        viz([NaN; NaN; NaN], waypointsGlobales);
    end
    
    % Bucle de movimiento para la facción actual
    while true
        % Calcular velocidades con Pure Pursuit
        [vRef, wRef] = controller(curPose);
        
        % Actualizar posición del robot (Cinemática Diferencial)
        velB = [vRef; 0; wRef];
        vel = bodyToWorld(velB, curPose);
        curPose = curPose + vel * sampleTime;
        
        % Actualizar ventana gráfica
        viz(curPose, waypointsGlobales);
        xlim([-10 10]);
        ylim([-10 10]);
        
        % Verificar si el robot terminó el trazo actual
        distAlFinal = norm(curPose(1:2) - puntosActuales(end,:)');
        if distAlFinal < 0.25
            break; % Rompe el bucle interno y salta a la siguiente facción
        end
        
        waitfor(r);
    end
end

fprintf('\n¡Dibujo finalizado por completo sin líneas cruzadas en el rostro!\n');