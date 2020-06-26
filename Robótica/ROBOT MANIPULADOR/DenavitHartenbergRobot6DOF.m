% LÓPEZ SÁNCHEZ, JAVIER | 70921610Y
% MATEOS PEDRAZA, ALEJANDRO | 70969732n

%Matrices de Transformación de Denavit-Hartenberg

% A [i-1 to i] = R(z, theta_i) * T(0,0,d_i) * T(a_i,0,0) * R(x, alpha_i)

% Rotación en z con un ángulo theta_i
% cos_theta   -sen_theta        0       0
% sen_theta   +cos_theta        0       0
%         0            0        1       0
%         0            0        0       1

% Traslación en z una distancia d_i
% 1 0 0 0
% 0 1 0 0
% 0 0 1 d 
% 0 0 0 1

% Traslación en x una distancia a_i
% 1 0 0 a
% 0 1 0 0
% 0 0 1 0
% 0 0 0 1

% Rotación en x un ángulo alpha_i
% 1         0            0   0
% 0 cos_alpha   -sen_alpha   0
% 0 sen_alpha   +cos_alpha   0
% 0         0            0   1


% Parámetros de transformación de nuestro Robot
%Between 'Art_1' and 'Art_2':
%    d=0.4200
%    theta=-0.0
%    a=0.1000
%    alpha=90.0

desplazamiento = 0.08;
angulo = 0;
d = 0.42 + desplazamiento;
theta = 0 + angulo;
a = 0.1;
alpha = 90;

DH_12 = DenavitHartenberg(d, theta, a, alpha);
disp("Transformación de Denavit Hartenberg: Articulación 1 -> Articulación 2");
disp(DH_12);

%Between 'Art_2' and 'Art_3':
%    d=0.0000
%    theta=0.0
%    a=0.2000
%    alpha=0.0

angulo = 0;
d = 0;
theta = 0 + angulo;
a = 0.2;
alpha = 0;

DH_23 = DenavitHartenberg(d, theta, a, alpha);
disp("Transformación de Denavit Hartenberg: Articulación 2 -> Articulación 3");
disp(DH_23);

%Between 'Art_3' and 'Art_4':
%    d=-0.0000
%    theta=0.0
%    a=0.4000
%    alpha=90.0

angulo = 0;
d = 0;
theta = 0 + angulo;
a = 0.4;
alpha = 90;

DH_34 = DenavitHartenberg(d, theta, a, alpha);
disp("Transformación de Denavit Hartenberg: Articulación 3 -> Articulación 4");
disp(DH_34);

%Between 'Art_4' and 'Art_5':
%    d=-0.0000
%    theta=-0.0
%    a=0.1000
%    alpha=90.0

angulo = 0;
d = 0;
theta = 0 + angulo;
a = 0.1;
alpha = 90;

DH_45 = DenavitHartenberg(d, theta, a, alpha);
disp("Transformación de Denavit Hartenberg: Articulación 4 -> Articulación 5");
disp(DH_45);

%Between 'Art_5' and 'Art_6':
%    d=0.0000
%    theta=0.0
%    a=0.3000
%    alpha=90.0

angulo = 0;
d = 0;
theta = 0 + angulo;
a = 0.3;
alpha = 90;

DH_56 = DenavitHartenberg(d, theta, a, alpha);
disp("Transformación de Denavit Hartenberg: Articulación 5 -> Articulación 6");
disp(DH_56);

%Between 'Art_6' and 'Final':
%    d=0.0000
%    theta=0.0
%    a=0.2000
%    alpha=0.0

angulo = 0;
d = 0;
theta = 0 + angulo;
a = 0.2;
alpha = 0;

DH_6F = DenavitHartenberg(d, theta, a, alpha);
disp("Transformación de Denavit Hartenberg: Articulación 6 -> Articulación Final");
disp(DH_6F);

%Matriz de Transformación Homogénea Final (DH_F):
DH_F = DH_12 * DH_23 * DH_34 * DH_45 * DH_56 * DH_6F;

disp("Transformación Homogénea Final de Denavit Hartenberg:");
disp(DH_F);

%FUNCIÓN PARA CALCULAR LAS TRANSFORMACIONES DE DENAVIT-HARTENBERG
function DH = DenavitHartenberg(d, theta, a, alpha)
theta = theta*pi/180;
alpha = alpha*pi/180;

Rz = [ cos(theta) -sin(theta) 0 0
       sin(theta)  cos(theta) 0 0
                0           0 1 0
                0           0 0 1  ];

Tz = [1 0 0 0 
      0 1 0 0
      0 0 1 d
      0 0 0 1];
  
Tx = [1 0 0 a
      0 1 0 0
      0 0 1 0
      0 0 0 1];

Rx = [1          0           0      0
      0 cos(alpha) -sin(alpha)      0
      0 sin(alpha)  cos(alpha)      0
      0          0           0      1 ];


%La matriz DH_12 es la matriz de transformación de art_1 a art_2 
DH = Rz * Tz * Tx * Rx;
end
