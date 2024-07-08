clear all;
close all;
clc;
fiy=1;

%Initialization
th_int=[-pi/2 pi/2];%Initial positions 
ths=[pi/2 -pi/2];%set points

x0=[0 0 th_int 0 0 0 0];%initial positions
Ts=[0 20];%time-span

%Robot specifications 
L1=1; %link 1
L2=1; %link 2
M1=1; %mass1
M2=1; %mass2
spec=[L1 L2 M1 M2];

%PID Parameters
%PID Parameters for Kpid1
Kp1=15;
Kd1=7;
Ki1=10;
%Parameters for Kpid2
Kp2=15;
Kd2=10;
Ki2=10;

Kpid=[Kp1 Kd1 Ki1 Kp2 Kd2 Ki2];

%ODE solving 
%opt1=odeset('RelTol', 1e-10, 'Abstrol', 1e-20, 'NormControl', 'off');
[T,X]= ode45(@(t,x) r2dof(t,x,ths,spec,Kpid), Ts, x0);

%Output 
th1=X(:,3); %theta1 waveform
th2=X(:,4); %theta2 waveform

%torque inputs computation from the 7th, 8th states inside ODE
F1=diff(X(:,7))./diff(T);
F2=diff(X(:,8))./diff(T);
tt=0:(T(end)/(length(F1)-1)):T(end);

%xy
x1=L1*cos(th1);
y1=L1*sin(th1);
x2=L1*cos(th1)+L2*cos(th1+th2);
y2=L1*sin(th1)+L2*sin(th1+th2);

%theta error plot
figure(fiy); fiy=fiy+1;
plot(T, ths(1)-th1);
grid;
title("Theta-1 error");
ylabel('theta error(rad)');
xlabel('T(sec)');
%theta2 error plot
figure(fiy); fiy=fiy+1;
plot(T, ths(2)-th2);
grid;
title("Theta-2 error");
ylabel('Theta error(rad)');
xlabel('T(sec)');
%torque1 plot
figure(fiy); fiy=fiy+1;
plot(tt, F1);
grid;
title('Torque of theta1');
xlabel('time (sec)');
ylabel('theta1 torque(rad)');
%torque 2 plot
figure(fiy); fiy=fiy+1;
plot(tt, F2);
grid;
title('Torque of theta2');
xlabel('time (sec)');
ylabel('theta2 torque (rad)');