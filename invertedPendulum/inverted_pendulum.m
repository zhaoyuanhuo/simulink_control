clear all; close all; clc;
%{
This MATLAB code is for learning inverted pendulum control problem.
Author: Zhaoyuan Huo
Email: zhuo2@andrew.cmu.edu
%}
%% Modeling
% Goals: 
%   settling time for theta < 5s
%   theta < 0.05 from the vertical, always
%{
F: control input, positive direction is to the right
x: horizontal position of the cart
m = 0.2kg: mass of the pendulum
M = 0.5kg: mass of the cart
I = 0.006kg.m^2: mass moment of inertia of the pendulum
b = 0.1N/m/sec: coeff of friction for the cart
l: pendulum COM pos to the joint
theta: angular position of the pendulum. 0=>downward, pi/2=>to the right,
pi=>upward; Initially theta0 = pi
%}
%{ 
(I+m*l^2)*ddphi - m*g*l*phi = m*l*ddx
(M+m)ddx + b*dx -m*l*ddphi = u
%}
%{
states:
x
dx
phi
dphi
%}


%% Params
M = 0.5;
m = 0.2;
b = 0.1;
I = 0.006;
g = 9.8;
l = 0.3;

%% TF Method
q = (M+m)*(I+m*l^2)-(m*l)^2;
s = tf('s');

P_cart = (((I+m*l^2)/q)*s^2 - (m*g*l/q))/(s^4 + (b*(I + m*l^2))*s^3/q - ((M + m)*m*g*l)*s^2/q - b*m*g*l*s/q);

P_pend = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);

sys_tf = [P_cart ; P_pend];

inputs = {'u'};
outputs = {'x'; 'phi'};

set(sys_tf, 'InputName', inputs);
set(sys_tf, 'outputName', outputs);

sys_tf

%% State Space Method
% p = I*(M+m)+M*m*l^2;
% 
% A = [0      1              0           0;
%      0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
%      0      0              0           1;
%      0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
% B = [     0;
%      (I+m*l^2)/p;
%           0;
%         m*l/p];
% C = [1 0 0 0;
%      0 0 1 0];
% D = [0;
%      0];
% 
% states = {'x' 'x_dot' 'phi' 'phi_dot'};
% inputs = {'u'};
% outputs = {'x'; 'phi'};
% 
% sys_ss = ss(A, B, C, D, 'statename', states, 'inputname', inputs, 'outputname', outputs)
% 
% sys_tf = tf(sys_ss) % convert ss to tf

%% examine open loop impulse response
t = 0:0.01:1;
impulse(sys_tf,t); % will show that the system is not stable in open loop
title('Open Loop Impulse Response');

%% examine poles and zeros
[zeros_pend poles_pend] = zpkdata(P_pend, 'v')
[zeros_cart poles_cart] = zpkdata(P_cart, 'v')
% the poles for both TFs are identical, and there is one unstable pole on
% the right half plane

%% examine open loop step response
t = 0:0.05:10;
u = ones(size(t));
[y,t] = lsim(sys_tf, u, t);
plot(t,y)
title('Open-Loop Step Response');
axis([0 3 0 50]);
legend('x', 'phi');

step_info = lsiminfo(y,t)
cart_info = step_info(1)
pend_info = step_info(2)





