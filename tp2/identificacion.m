clear all;
clc;
load('datos_planta_actualizado.mat');

%% Preprocesamiento de datos.

phi = out.d1; %Se cargan los datos leídos mediante el potenciómetro.
theta = out.d2; %Se cargan los datos leídos por la IMU.
t= out.tout; %Se cargan los tiempos.
Ts=t(100) - t(99); %Se define la tasa de muestreo como los 20ms

% Se recorta el primer período, que corresponde a un movimiento a 45°
% positivo.
theta=theta(1731:2016); %Se le resta la media
theta=theta-mean(theta);
t=t(1731:2016)-t(1731);
phi=phi(1731:2016);

% Se setean a 45.8358° a partir de la muestra 193
phi(216:end) = -43.1936;

% % Las primeras y las ultimas 15 muestras de theta, se setean a 0
theta(1)=0;
theta(end-5:end) = 0;

%% Estimación del modelo.

% Suponiendo theta como la salida y phi como la entrada.
data = iddata(double(theta(:)),double(phi(:)));
np=4; % cantidad de polos.
nz=3; % cantidad de ceros.
G_est = tfest(data,np,nz); % Modelo en tiempo continuo

% Validación de la estimación del modelo.
compare(data,G_est);

% La planta es estable.
p=[
  -0.5809 + 0.0000i
  -0.0092 + 0.1573i
  -0.0092 - 0.1573i
  -0.0650 + 0.0000i
    ];

z= [
    0
   -0.0543
   -0.0047
   ];

k = -0.6942;

% Se establecen los polos, ceros y ganancia finales del sistema estimado.
% Y se agrega el Padé correspondiente al retardo de media muestra.
Gap = zpk([200],[-200],-1);
G = zpk(z,p,k)*Gap;

% Se valida la estimación final del sistema.
figure();
compare(data,G);

%% Diseño del controlador.
k = db2mag(0);
C = zpk([-0.0151 -0.0151],[0],k);

Cd = c2d(C,Ts,'tustin');

margin(G*C)
OL = G*C;
CL = OL/(1+OL);
