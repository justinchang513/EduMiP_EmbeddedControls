% MAE 144 EduMIP Project Part 1
% Group 8 (Justin Chang, Khang, Basil)
 
%% Given Parameters
clear all; clc; close all;

mw      = 0.027;
mb      = 0.180;
wf      = 1760;
sbar    = 0.003;
G       = 320/9;
Im      = 3.6*10^-8;
R       = 0.034;
L       = 0.0477;  
Ib      = 2.63*10^-4;
Vn      = 7.4;

DT1     = 0.01; %100Hz sample rate
DT2     = 0.05; %20Hz sample rate
gravity = 9.81;
 
% wheel inertia
Iw = 2*(mw*R^2/2+G^2*Im);
 
%% Plant & Controller TFs
a = Iw + (mw+mb)*R^2;
b = 2*(G^2)*sbar/wf;
c = mb *R*L;
d = 2*G*sbar;
e = mb*R*L;
f = Ib + mb*(L^2);
g = mb*gravity*L;

%% Inner Loop
numG1 = [d*(e+a) 0 0];
denG1 = [(e*c-a*f) (-e*b-b*c-a*b-b*f) (a*g) (b*g) 0];
G1 = tf(numG1,denG1);
G1 = minreal(G1);

%System's Root Locus
figure(1);
rlocus(G1);

%Controller design D1
k = -1;
D1 = tf([1 8],[1 0]);
D1 = D1*k;

% Discretization
 [numD1, denD1] = tfdata(D1,'v');
 [numD1z ,denD1z] = c2d(D1,DT1,'Tustin');
 D1z = tf(numD1z,denD1z,DT1);

%Root locus after controller
figure(2);
rlocus(G1*D1)

%Bode Plot G1*D1
figure(3);
margin(G1*D1)

%Step response
 figure(4);
 T = G1*D1/(1+G1*D1);
 step(T)
 

%% Outer Loop
numG2 = [(-c-f) 0 (g)];
denG2 = [(a+e) 0 0];
G2 = tf(numG2,denG2);
G2 = minreal(G2);

% Ideal system without controller
figure(5)
rlocus(G2) 

%D2 Controller
 K2      = 1;
extrapole= tf([1],[1 7]);
PD       = tf([1 0],[1 13]);
D2       = K2*extrapole*PD;


% Ideal system with outer loop controller
figure(6)
rlocus(G2*D2) 

% Ideal Step response
figure(7)
step(G2*D2/(1+G2*D2));
ylim([0,1.3]);

%Realistic step response
figure(8)
step(G2*D2*G1*D1*(1/1.48)/((1+G1*D1)*(1+G2*D2)));
hold on;
step(T);
hold off;
legend('Outer Loop Step Response','Inner Loop Step Response');
xlim([0,4]);

% Open-Loop Bode Plot
figure(9)
margin(G2*D2);

%Open-Loop Bode Plot with Inner Loop Model Included
figure(10)
margin(G2*D2*T/1.48);

%Discretization
[numD2, denD2] = tfdata(D2,'v');
[numD2z, denD2z] = c2d(D2,DT2,'Tustin');
D2z = tf(numD2z,denD2z,DT2);