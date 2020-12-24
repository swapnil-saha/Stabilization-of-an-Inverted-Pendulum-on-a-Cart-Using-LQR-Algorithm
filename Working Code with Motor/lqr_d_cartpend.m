clear all, close all, clc

%parameters for the system
m = 1;
M = 5;
L = 2;
g = -10;
d = 1;

s = 1; % pendulum up (s=1)

% derivative(X) = A.X + B.u
% y = C.X + D.u
% X = [linear position;
%      linear velocity;
%      angular position;
%      angular velocity]

A = [0 1 0 0;
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];

B = [0; 1/M; 0; s*1/(M*L)];

C = [1 1 1 1]; %C is denoted in this way to build the discrete version
%of original state function response from output y
D = 0;

% Q puts a penalty on X
% R puts a penalty on u
Q = [1 0 0 0;
    0 1 0 0;
    0 0 10 0;
    0 0 0 100];
R = .01;

%%
%to see if the system is controllable and observable
rank(ctrb(A,B))
rank(obsv(A,C))
%%
K = lqr(A,B,Q,R);   %lqr control of system in continuos time domain

%%
sys = ss(A,B,C,D);  %state space model of system

Ts = 0.1;           %sampling time in seconds
sys_d = c2d(sys,Ts);%continuous to discrete conversion

%discrete version of A,B,C and D
A_d = sys_d.A;
B_d = sys_d.B;
C_d = sys_d.C;
D_d = sys_d.D;

%N has no function, therefore it is declared as zero matrix
N = zeros(length(B),1);

[Kd,~,~] = dlqr(A_d,B_d,Q,R,N);
[Obd,~,~] = dlqr(A_d',C_d',Q,R,N);

%%
tspan = 0:.001:20;
if(s==-1)
    y0 = [-3; 0; 0; 0];
    [t,y] = ode45(@(t,y)cartpend(y,-K*(y-[4; 0; 0; 0])),tspan,y0);
elseif(s==1)
    y0 = [-3; 0; pi + pi/4; 0];
    [t,y] = ode45(@(t,y)cartpend(y,-K*(y-[3; 0; pi; 0])),tspan,y0);
else
    
end

for k=1:100:length(t)
    drawcartpend_bw(y(k,:));
end