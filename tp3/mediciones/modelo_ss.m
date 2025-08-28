clear; clc; close all;

load('datos_planta_actualizado.mat');

phi = out.d1;
theta = out.d2;
t = out.tout;

% Preprocesamiento de datos.
phi = phi(1731:2016)- mean(phi);
theta = theta(1731:2016);
theta = theta - mean(theta);
t = t(1731:2016) - t(1731);

phi_ref = -45; % Para las mediciones, la referencia está en grados.

% Se definen entradas y salidas del sistema.
u = phi_ref * ones(length(t), 1);         
y = [double(phi(:)) double(theta(:))]; 
Ts = 0.02;

% Se genera la matriz de datos, con las etiquetas de entradas y salidas.
data = iddata(y, u,Ts);  % y = [phi, theta], u = phi_ref
data.InputName = {'phi_ref'};
data.OutputName = {'phi', 'theta'};

order = 4;  % Orden del sistema, que equivale a la cantidad de variables de estado.

% Se estima el sistema, considerando la matriz de datos y el orden
% estipulado.
sys = ssest(data, order); % Sistema en tiempo continuo.

sys_d = c2d(sys ,Ts, 'tustin'); %Discretización del sistema.

% Se obtienen las matrices del sistema discretizado, para la obtención de
% las ganancias del observador y el diseño del controlador.
A = sys_d.A;
B = sys_d.B;
C = sys_d.C;
D = sys_d.D;

% Se verifica el ajuste continuo y la discretización del sistema.
% Se comparan las salidas estimadas vs las mediciones de la planta

% Obtener la respuesta simulada y las mediciones desde compare
[y_sim_id, fit, ~] = compare(data, sys_d);

% y_sim es un iddata con las señales simuladas
% data es el iddata con las señales medidas

figure();
plot(t, data.OutputData(:,1), 'LineWidth', 1.5, 'DisplayName', 'Medición')
hold on
plot(t, y_sim_id.OutputData(:,1), '--', ...
'LineWidth', 1.5, 'DisplayName', sprintf('Modelo (%.1f%%)', fit(1)))
legend('Location', 'northeast')
grid on
title('Medición vs Estimación (\phi)')
xlabel('Time (s)')
ylabel('Amplitude (degrees)')
ticks_x = 0:1:6;
ticks_y = -50:25:50;
xticks(ticks_x)
yticks(ticks_y)
ylim([-50, 50])
xlim([0,5.7])

figure();
plot(t, data.OutputData(:,2), 'LineWidth', 1.5, 'DisplayName', 'Medición')
hold on
plot(t, y_sim_id.OutputData(:,2), '--', ...
'LineWidth', 1.5, 'DisplayName', sprintf('Modelo (%.1f%%)', fit(2)))
legend('Location', 'northeast')
grid on
title('Medición vs Estimación (\theta)')
xlabel('Time (s)')
ylabel('Amplitude (degrees)')
ticks_x = 0:1:6;
ticks_y = -60:20:60;
xticks(ticks_x)
yticks(ticks_y)
ylim([-60, 60])
xlim([0,5.7])




%Se verifica la observabilidad y controlabilidad del sistema.
% Matriz de controlabilidad.
Co = ctrb(A, B);
rank_Co = rank(Co);          % Rango de la matriz = orden del sistema.

% Matriz de observabilidad.
Ob = obsv(A, C);
rank_Ob = rank(Ob);          % Rango de la matriz = orden del sistema.




%% Diseño del observador discreto
% Se acomodan los datos como filas.
u = u';
y = y';

% Se obtienen los polos del sistema y la cantidad de muestras.
poles_sys = eig(A);
N = size(u, 2); % Número de muestras.

% Se asignan los polos del observador.
poles_obs = poles_sys *0.1

% Se compueba que los polos del observador asignados, se encuentran dentro
% del círculo unitario.
poles_obs(abs(poles_obs) >= 1) = 0.9;

% Se obtiene la ganancia del observador.
L = place(A', C', poles_obs)';

% Se simula el observador discreto.
x_hat = zeros(size(A,1),1);
x_hat_hist = zeros(size(A,1), N);
y_hat_hist = zeros(size(C,1), N);


for k = 1:N-1
    y_hat = C * x_hat + D * u(:,k);
    innov = y(:,k) - y_hat;
    x_hat = A * x_hat + B * u(:,k) + L * innov;
    x_hat_hist(:, k+1) = x_hat;
    y_hat_hist(:, k) = y_hat; 
end

t_plot = t(1:N-1); % Se adapta t.

% Se grafican las salidas del sistema estimadas con el observador.
figure();
plot(t_plot, y(1,1:end-1), 'b', 'DisplayName', 'Medición');
hold on;
plot(t_plot, y_hat_hist(1,1:N-1), 'r--', 'DisplayName', 'Observador');
title('Estimación del observador vs Medición (\phi)')
xlabel('Time (s)');
ylabel('Amplitude (degrees)');
legend('Location', 'northeast')
xlim([0,5.7])
grid on;

figure();
plot(t_plot, y(2,1:end-1), 'b', 'DisplayName', 'Medición');
hold on;
plot(t_plot, y_hat_hist(2,1:N-1), 'r--', 'DisplayName', 'Observador');
title('Estimación del observador vs Medición (\theta)')
xlabel('Time (s)');
ylabel('Amplitude (degrees)');
legend('Location', 'northeast')
xlim([0,5.7])
grid on;

%% Diseño del controlador discreto.

% Se ubican los polos del lazo cerrado, considerando que deben encontrarse
% dentro de la circunferencia unitaria, para asegurar estabilidad.
poles_control = [0.7, 0.9, 0.6, 0.8]; 
% Son alejados del origen, para evitar el sobrepico en el control.

% Se obtiene la ganancia para el control por realimentación de estados.
K = place(A, B, poles_control);

% Acción de control, utilizando los estados estimados por el observador.
u = -K * x_hat_hist; 

%% Diseño del controlador discreto, con seguimiento de referencias.

% Se obtiene la matriz de feedforward, utilizando una estimación por
% cuadrados mínimos, ya que se tienen 2 salidas y 1 entrada.
% Si se quiere siempre theta = 0 y cambiar phi_ref: MFF = [6    0]
Mff = pinv(C * inv(eye(size(A)) - A + B*K) * B);

r= [0;0]; % Referencias.
% Acción de control, utilizando los estados estimados por el observador.
u = -K * x_hat_hist + Mff * r;


%% Comprobación del seguimiento de referencias + acción integral
%---------- Parámetros del sistema 
n = size(A,1);  % Número de estados
p = size(C,1);  % Número de salidas
m = size(B,2);  % Número de entradas

% Se extrae la fila correspondiente a theta
C_theta = C(2,:);  

% Ampliación del sistema: acción integral solo sobre theta
A_ext = [A           zeros(n,1);
        -C_theta     0];
B_ext = [B;
         0];

%------------------ Se verifica la controlabilidad del sistema a lazo cerrado,
% con acción integral para la salida theta.
Co = ctrb(A_ext, B_ext);
rango = rank(Co);
disp(['Rango de la matriz de controlabilidad: ', num2str(rango)]);
disp(['¿Es controlable? ', string(rango == size(A_ext,1))]);


Ki = 0.1;    % Ganancia del integrador

% Simulación
r_phi   = 20;     % referencia en phi
r_theta = 0;    % referencia en theta
r = [r_phi; r_theta];

x = zeros(n,1);      % Estados originales
xi = 0;              % Estado del integrador
N = 1000;             % Duración de la simulación
y_hist = zeros(p,N); % Salidas
u_hist = zeros(1,N); % Entradas

for k = 1:N
    y = C*x;
    e_theta = r(2) - y(2);  % Solo el error en theta
    xi = xi + e_theta;      % Integrador

    u = -K*x - Ki*xi;       % Control con acción integral solo sobre theta
    x = A*x + B*u;

    y_hist(:,k) = y;
    u_hist(k) = u;
end

% Gráfica
figure();
time = 1:N;
plot(time, y_hist(1,:), 'b-', ...
     time, y_hist(2,:), 'r-', ...
     time, repmat(r(1),1,N), 'b--', ...
     time, repmat(r(2),1,N), 'r--');
legend('\phi','\theta','Ref \phi','Ref \theta');
xlabel('Tiempo (pasos)');
ylabel('Salida');
title('Respuesta con acción integral solo en \theta');
grid on;

close all;
clear all;
clc;

%% OBSERVADOR

%% phi
datos = load('mediciones_observador.mat');
load ('simulacion_observador.mat');

% Datos de simulacion
t = simulacion_observador.Time;
phi = simulacion_observador.Data(:,1);

phi_medicion = datos.out.d1 ;  
phi_medicion = phi_medicion(236:741);
% plot(t, phi_medicion)

% Crear la figura
% t2=0:0.02:10.12;
figure
plot(t, phi, 'b-', 'LineWidth', 1); hold on
plot(t, phi_medicion(1:end-5), 'r-', 'LineWidth', 1);
grid on

% Etiquetas y título con LaTeX
xlabel('Time (s)', 'FontSize', 8, 'FontWeight', 'bold')
ylabel('Amplitude (degree)', 'FontSize', 8, 'FontWeight', 'bold')

% Leyenda con LaTeX correctamente aplicada
legend('simulación', 'medición','Location','southeast')

% Estilo de la gráfica
set(gca, 'FontSize', 8)
set(gcf, 'Color', 'w')

title('Comparación del ángulo \phi para el observador')

% Guardar imagen con alta resolución
print('comparacion_phi','-dpng','-r600')


%% theta
datos = load('mediciones_observador.mat');
load ('simulacion_observador.mat');

% Datos de simulacion
t = simulacion_observador.Time;
theta = simulacion_observador.Data(:,2);

theta_medicion = datos.out.d2 ;  
theta_medicion = theta_medicion(236:741);
% plot(t, phi_medicion)

% Crear la figura
figure
plot(t, theta, 'b-', 'LineWidth', 1); hold on
plot(t, theta_medicion(1:end-5), 'r-', 'LineWidth', 1);
grid on

% Etiquetas y título con LaTeX
xlabel('Time (s)', 'FontSize', 8, 'FontWeight', 'bold')
ylabel('Amplitude (degree)', 'FontSize', 8, 'FontWeight', 'bold')

% Leyenda con LaTeX correctamente aplicada
legend('simulación', 'medición')

% Estilo de la gráfica
set(gca, 'FontSize', 8)
set(gcf, 'Color', 'w')

title('Comparación del ángulo \theta para el observador')

% Guardar imagen con alta resolución
print('comparacion_theta','-dpng','-r600')


%% phi_punto
datos = load('mediciones_observador.mat');
load ('simulacion_observador.mat');

% Datos de simulacion
t = simulacion_observador.Time;
phi = simulacion_observador.Data(:,1);

phi_medicion = datos.out.d1 ;  
phi_medicion = phi_medicion(236:741);
% plot(t, phi_medicion)

phi_dot = diff(phi)/0.02;

phi_dot_medicion = diff(phi_medicion)/0.02;
% Crear la figura
% t2=0:0.02:10.12;
figure
plot(t(1:end-1), phi_dot, 'b-', 'LineWidth', 1); hold on
plot(t(1:end-1), phi_dot_medicion(1:end-5), 'r-', 'LineWidth', 1);
grid on

% Etiquetas y título con LaTeX
xlabel('Time (s)', 'FontSize', 8, 'FontWeight', 'bold')
ylabel('Velocity (degree/s)', 'FontSize', 8, 'FontWeight', 'bold')

% Leyenda con LaTeX correctamente aplicada
legend('simulación', 'medición')

% Estilo de la gráfica
set(gca, 'FontSize', 8)
set(gcf, 'Color', 'w')

title('Comparación de la derivada de \phi para el observador')

% Guardar imagen con alta resolución
print('comparacion_phi_dot','-dpng','-r600')


%% theta_punto
datos = load('mediciones_observador.mat');
load ('simulacion_observador.mat');

% Datos de simulacion
t = simulacion_observador.Time;
theta = simulacion_observador.Data(:,2);

theta_medicion = datos.out.d2 ;  
theta_medicion = theta_medicion(236:741);
% plot(t, phi_medicion)

theta_dot = diff(theta)/0.02;

theta_dot_medicion = diff(theta_medicion)/0.02;
% Crear la figura
% t2=0:0.02:10.12;
figure
plot(t(1:end-1), theta_dot, 'b-', 'LineWidth', 1); hold on
plot(t(1:end-1), theta_dot_medicion(1:end-5), 'r-', 'LineWidth', 1);
grid on

% Etiquetas y título con LaTeX
xlabel('Time (s)', 'FontSize', 8, 'FontWeight', 'bold')
ylabel('Velocity (degree/s)', 'FontSize', 8, 'FontWeight', 'bold')

% Leyenda con LaTeX correctamente aplicada
legend('simulación', 'medición')

% Estilo de la gráfica
set(gca, 'FontSize', 8)
set(gcf, 'Color', 'w')

title('Comparación de la derivada de \theta para el observador')

% Guardar imagen con alta resolución
print('comparacion_theta_dot','-dpng','-r600')



%% SIN REFERENCIA
%% PHI

datos = load('mediciones_y_realimentacion.mat');
load ('simulacion_realimentacion_sin_ref.mat');

% Datos de simulacion
t = datos_realimentacion_sin_ref.Time;
phi = datos_realimentacion_sin_ref.Data(:,1);

phi_medicion = datos.out.d1 ;  
phi_medicion = phi_medicion(3286:3541);

t3=0:0.02:5.12; 
% Crear la figura
figure
plot(t(1:end-15), phi(1:end-15), 'b-', 'LineWidth', 1); hold on
plot(t3(1:end-16), phi_medicion(1:end-15), 'r-', 'LineWidth', 1);
grid on
xlim([0,5]);

% Etiquetas y título con LaTeX
xlabel('Time (s)', 'FontSize', 8, 'FontWeight', 'bold')
ylabel('Amplitude (degrees)', 'FontSize', 8, 'FontWeight', 'bold')

% Leyenda con LaTeX correctamente aplicada
legend('simulación', 'medición')

% Estilo de la gráfica
set(gca, 'FontSize', 8)
set(gcf, 'Color', 'w')

title('Comparación de \phi para el controlador sin referencias.')

% Guardar imagen con alta resolución
print('comparacion_phi_realimentacion_sin_ref','-dpng','-r600')

%% THETA

datos = load('mediciones_y_realimentacion.mat');
load ('simulacion_realimentacion_sin_ref.mat');

% Datos de simulacion
t = datos_realimentacion_sin_ref.Time;
theta = datos_realimentacion_sin_ref.Data(:,2);

theta_medicion = datos.out.d2 ;  
theta_medicion = theta_medicion(3286:3541);

t3=0:0.02:5.12; 
% Crear la figura
figure
plot(t(1:end-15), theta(1:end-15), 'b-', 'LineWidth', 1); hold on
plot(t3(1:end-16), theta_medicion(1:end-15), 'r-', 'LineWidth', 1);
grid on
xlim([0,5]);

% Etiquetas y título con LaTeX
xlabel('Time (s)', 'FontSize', 8, 'FontWeight', 'bold')
ylabel('Amplitude (degrees)', 'FontSize', 8, 'FontWeight', 'bold')

% Leyenda con LaTeX correctamente aplicada
legend('simulación', 'medición')

% Estilo de la gráfica
set(gca, 'FontSize', 8)
set(gcf, 'Color', 'w')

title('Comparación de \theta para el controlador sin referencias.')

% Guardar imagen con alta resolución
print('comparacion_theta_realimentacion_sin_ref','-dpng','-r600')

%% PHI_DOT

datos = load('mediciones_y_realimentacion.mat');
load ('simulacion_realimentacion_sin_ref.mat');

% Datos de simulacion
t = datos_realimentacion_sin_ref.Time;
phi = datos_realimentacion_sin_ref.Data(:,1);

phi_medicion = datos.out.d1 ;  
phi_medicion = phi_medicion(3286:3541);

phi_dot_sim = diff(phi)/0.02;
phi_dot_med = diff(phi_medicion)/0.02;

t3=0:0.02:5.12; 
% Crear la figura
figure
plot(t(1:end-16), phi_dot_sim(1:end-15), 'b-', 'LineWidth', 1); hold on
plot(t3(1:end-17), phi_dot_med(1:end-15), 'r-', 'LineWidth', 1);
grid on
xlim([0,5]);

% Etiquetas y título con LaTeX
xlabel('Time (s)', 'FontSize', 8, 'FontWeight', 'bold')
ylabel('Velocity (degrees/s)', 'FontSize', 8, 'FontWeight', 'bold')

% Leyenda con LaTeX correctamente aplicada
legend('simulación', 'medición')

% Estilo de la gráfica
set(gca, 'FontSize', 8)
set(gcf, 'Color', 'w')

title('Comparación de la derivada de \phi para el controlador sin referencias.')

% Guardar imagen con alta resolución
print('comparacion_phi_dot_realimentacion_sin_ref','-dpng','-r600')

%% THETA_DOT

datos = load('mediciones_y_realimentacion.mat');
load ('simulacion_realimentacion_sin_ref.mat');

% Datos de simulacion
t = datos_realimentacion_sin_ref.Time;
theta = datos_realimentacion_sin_ref.Data(:,2);

theta_medicion = datos.out.d2 ;  
theta_medicion = theta_medicion(3286:3541);

theta_dot_sim = diff(theta)/0.02;
theta_dot_med = diff(theta_medicion)/0.02;

t3=0:0.02:5.12; 
% Crear la figura
figure
plot(t(1:end-16), theta_dot_sim(1:end-15), 'b-', 'LineWidth', 1); hold on
plot(t3(1:end-17), theta_dot_med(1:end-15), 'r-', 'LineWidth', 1);
grid on
xlim([0,5]);

% Etiquetas y título con LaTeX
xlabel('Time (s)', 'FontSize', 8, 'FontWeight', 'bold')
ylabel('Velocity (degrees/s)', 'FontSize', 8, 'FontWeight', 'bold')

% Leyenda con LaTeX correctamente aplicada
legend('simulación', 'medición')

% Estilo de la gráfica
set(gca, 'FontSize', 8)
set(gcf, 'Color', 'w')

title('Comparación de la derivada de \theta para el controlador sin referencias.')

% Guardar imagen con alta resolución
print('comparacion_theta_dot_realimentacion_sin_ref','-dpng','-r600')


%% GRAFICOS DE REALIMENTACION CON CONTROL DE TRAYECTORIAS Y DE CONTROL CON ACCION INTEGRAL

%% PHI  Realimentacion de estados con referencia
datos = load('mediciones_y_con_ref.mat');
load ('simulacion_realimentacion_con_ref.mat');

% Datos de simulacion
t = datos_realimentacion_con_ref.Time;
phi = datos_realimentacion_con_ref.Data(:,1);

% Datos de medición
phi_medicion = datos.out.d1 ;
phi_medicion = phi_medicion(1238:1742); %recorte de 10.08 segs

% Crear la figura
figure
plot(t(1:end-15), phi(1:end-15), 'b-', 'LineWidth', 1); hold on
plot(t(1:end-15), phi_medicion(1:end-15), 'r-', 'LineWidth', 1);
grid on

% Etiquetas y título con LaTeX
xlabel('Time (s)', 'FontSize', 8, 'FontWeight', 'bold')
ylabel('Amplitude (degrees)', 'FontSize', 8, 'FontWeight', 'bold')

% Leyenda con LaTeX correctamente aplicada
legend('simulación', 'medición', 'Location', 'southeast')

% Estilo de la gráfica
set(gca, 'FontSize', 8)
set(gcf, 'Color', 'w')

title('Comparación de \phi para el controlador con referencias')

% Guardar imagen con alta resolución
print('comparacion_phi_realimentacion_con_ref','-dpng','-r600')

%% THETA Realimentacion de estados con referencia
datos = load('mediciones_y_con_ref.mat');
load ('simulacion_realimentacion_con_ref.mat');

% Datos de simulacion
t = datos_realimentacion_con_ref.Time;
theta = datos_realimentacion_con_ref.Data(:,2);

theta_medicion = datos.out.d2 ;

theta_medicion = theta_medicion(1238:1742);
% plot( datos.out.tout , theta_medicion)

% % Crear la figura
figure;
plot(t(1:end-15), theta(1:end-15), 'b-', 'LineWidth', 1); hold on
plot(t(1:end-15), theta_medicion(1:end-15), 'r-', 'LineWidth', 1);
grid on

% Etiquetas y título con LaTeX
xlabel('Time (s)', 'FontSize', 8, 'FontWeight', 'bold')
ylabel('Amplitude (degrees)', 'FontSize', 8, 'FontWeight', 'bold')

% Leyenda con LaTeX correctamente aplicada
legend('simulación', 'medición')

% Estilo de la gráfica
set(gca, 'FontSize', 8)
set(gcf, 'Color', 'w')

title('Comparación de \theta para el controlador con referencias')

% Guardar imagen con alta resolución
print('comparacion_theta_realimentacion_con_ref','-dpng','-r600')


%% PHI_DOT  Realimentacion de estados con referencia
Ts = 0.02;
datos = load('mediciones_y_con_ref.mat');
load ('simulacion_realimentacion_con_ref.mat');

% Datos de simulacion
t = datos_realimentacion_con_ref.Time;
phi = datos_realimentacion_con_ref.Data(:,1);

phi_dot_simulacion = diff(phi) / Ts;

phi_medicion = datos.out.d1 ;     
phi_medicion = phi_medicion(1238:1742);

phi_dot_medicion = diff(phi_medicion) / Ts; 

% Crear la figura
figure
plot(t(1:end-16), phi_dot_simulacion(1:end-15), 'b-', 'LineWidth', 1); hold on
plot(t(1:end-16), phi_dot_medicion(1:end-15), 'r-', 'LineWidth', 1);
grid on

% Etiquetas y título con LaTeX
xlabel('Time (s)', 'FontSize', 8, 'FontWeight', 'bold')
ylabel('Velocity (degrees/s)', 'FontSize', 8, 'FontWeight', 'bold')

% Leyenda con LaTeX correctamente aplicada
legend('simulación', 'medición')

% Estilo de la gráfica
set(gca, 'FontSize', 8)
set(gcf, 'Color', 'w')

title('Comparación de la derivada de \phi para el controlador con referencias')

% Guardar imagen con alta resolución
print('comparacion_phi_dot_realimentacion_con_ref','-dpng','-r600')

%% THETA_DOT  Realimentacion de estados con referencia
Ts = 0.02;
datos = load('mediciones_y_con_ref.mat');
load ('simulacion_realimentacion_con_ref.mat');

% Datos de simulacion
t = datos_realimentacion_con_ref.Time;
theta = datos_realimentacion_con_ref.Data(:,2);

theta_dot_simulacion = diff(theta) / Ts;

theta_medicion = datos.out.d2 ;     
theta_medicion = theta_medicion(1238:1742);

theta_dot_medicion = diff(theta_medicion) / Ts; 

% % Crear la figura

t=t(12:end-16) - t(12);
figure
plot(t, theta_dot_simulacion(12:end-15), 'b-', 'LineWidth', 1); hold on
plot(t, theta_dot_medicion(12:end-15), 'r-', 'LineWidth', 1);
grid on

% Etiquetas y título con LaTeX
xlabel('Time (s)', 'FontSize', 8, 'FontWeight', 'bold')
ylabel('Velocity (degrees/s)', 'FontSize', 8, 'FontWeight', 'bold')

% Leyenda con LaTeX correctamente aplicada
legend('simulación', 'medición')

% Estilo de la gráfica
set(gca, 'FontSize', 8)
set(gcf, 'Color', 'w')

title('Comparación de la derivada de \theta para el controlador con referencias')

% Guardar imagen con alta resolución
print('comparacion_theta_dot_realimentacion_con_ref','-dpng','-r600')

%% PHI  Accion integral
datos = load('mediciones_accion_integral.mat');
load ('simulacion_accion_integral.mat')
% Datos simulacion
t = datos_accion_integral.Time;
phi = datos_accion_integral.Data(:,1);

phi_medicion = datos.out.d1 ;
phi_medicion = phi_medicion(1248:1748);

t2=0:0.02:10;
% % Crear la figura
figure
plot(t(1:end-15), phi(1:end-15), 'b-', 'LineWidth', 1); hold on
plot(t2(1:end-15), phi_medicion(1:end-15), 'r-', 'LineWidth', 1);
grid on

% Etiquetas y título con LaTeX
xlabel('Time (s)', 'FontSize', 8, 'FontWeight', 'bold')
ylabel('Amplitude (degrees)', 'FontSize', 8, 'FontWeight', 'bold')

% Leyenda con LaTeX correctamente aplicada
legend('simulación', 'medición', 'Location', 'southeast')

% Estilo de la gráfica
set(gca, 'FontSize', 8)
set(gcf, 'Color', 'w')

title('Comparación de \phi para el controlador con acción integral')

% Guardar imagen con alta resolución
print('comparacion_phi_accion_integral','-dpng','-r600')

%% THETA accion integral
datos = load('mediciones_accion_integral.mat');
load ('simulacion_accion_integral.mat')
% Datos simulacion
t = datos_accion_integral.Time;
theta = datos_accion_integral.Data(:,2);

theta_medicion = datos.out.d2 ;
theta_medicion = theta_medicion(1248:1748);

t2=0:0.02:10;
% % % Crear la figura
figure
plot(t(1:end-15), theta(1:end-15), 'b-', 'LineWidth', 1); hold on
plot(t2(1:end-15), theta_medicion(1:end-15), 'r-', 'LineWidth', 1);
grid on

% Etiquetas y título con LaTeX
xlabel('Time (s)', 'FontSize', 8, 'FontWeight', 'bold')
ylabel('Amplitude (degrees)', 'FontSize', 8, 'FontWeight', 'bold')

% Leyenda con LaTeX correctamente aplicada
legend('simulación', 'medición')

% Estilo de la gráfica
set(gca, 'FontSize', 8)
set(gcf, 'Color', 'w')

title('Comparación de \theta para el controlador con acción integral')

% Guardar imagen con alta resolución
print('comparacion_theta_accion_integral','-dpng','-r600')

%% PHI_DOT  Accion integral
Ts=0.02;
datos = load('mediciones_accion_integral.mat');
load ('simulacion_accion_integral.mat')
% Datos simulacion
t = datos_accion_integral.Time;
phi = datos_accion_integral.Data(:,1);

phi_medicion = datos.out.d1 ;
phi_medicion = phi_medicion(1248:1748);

phi_dot_simulacion = diff(phi)/Ts; 
phi_dot_medicion = diff(phi_medicion)/Ts;

t2=0:0.02:10;
% % Crear la figura
figure
plot(t(1:end-16), phi_dot_simulacion(1:end-15), 'b-', 'LineWidth', 1); hold on
plot(t2(1:end-16), phi_dot_medicion(1:end-15), 'r-', 'LineWidth', 1);
grid on

% Etiquetas y título con LaTeX
xlabel('Time (s)', 'FontSize', 8, 'FontWeight', 'bold')
ylabel('Velocity (degrees/s)', 'FontSize', 8, 'FontWeight', 'bold')

% Leyenda con LaTeX correctamente aplicada
legend('simulación', 'medición')

% Estilo de la gráfica
set(gca, 'FontSize', 8)
set(gcf, 'Color', 'w')

title('Comparación de la derivada de \phi para el controlador con acción integral')

% Guardar imagen con alta resolución
print('comparacion_phi_dot_accion_integral','-dpng','-r600')

%% THETA_DOT Accion integral
datos = load('mediciones_accion_integral.mat');
load ('simulacion_accion_integral.mat')
% Datos simulacion
t = datos_accion_integral.Time;
theta = datos_accion_integral.Data(:,2);

theta_medicion = datos.out.d2 ;
theta_medicion = theta_medicion(1248:1748);

theta_dot_simulacion = diff(theta)/Ts;
theta_dot_medicion = diff(theta_medicion) / Ts;

t=t(8:end-16)-t(8);
t2=0:0.02:9.56;
% % % Crear la figura
figure
plot(t, theta_dot_simulacion(8:end-15), 'b-', 'LineWidth', 1); hold on
plot(t2(1:end-1), theta_dot_medicion(8:end-15), 'r-', 'LineWidth', 1);
grid on

% Etiquetas y título con LaTeX
xlabel('Time (s)', 'FontSize', 8, 'FontWeight', 'bold')
ylabel('Velocity (degrees/s)', 'FontSize', 8, 'FontWeight', 'bold')

% Leyenda con LaTeX correctamente aplicada
legend('simulación', 'medición')

% Estilo de la gráfica
set(gca, 'FontSize', 8)
set(gcf, 'Color', 'w')

title('Comparación de la derivada de \theta para el controlador con acción integral')

% Guardar imagen con alta resolución
print('comparacion_theta_dot_accion_integral','-dpng','-r600')
