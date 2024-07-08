function xdot=r3dof(t, x, ths, spec, Kpid)
    xdot=zeros(12,1);

    %set-points
    th1s=taf(t,5,ths(1));
    th2s=taf(t,5,ths(2));
    th3s=taf(t,5,ths(3));
    
    %Robot specifications
    g=9.8;
    f1=7e-4;f2=f1; f3=100;
    k1=0.18; k2=k1; k3=k1;
    
    %Inertia Parameters
    a=0.5*((1/3)*spec(4)+spec(5)+spec(6))*spec(1)^2;
    b=((7/24)*spec(5)+0.5*spec(6))*spec(2)^2;
    c=(0.5*spec(5)+spec(6))*spec(1)*spec(2);
    d=0.5*spec(6);
    e=spec(6)*g;

    %Inertia Matrix
    b11=2*a+2*b+2*c*cos(x(5));
    b12=2*b+c*cos(x(5));
    b13=0;
    b21=2*b+c*cos(x(5));
    b22=2*b;
    b23=0;
    b31=0; b32=0;
    b33=2*d;
    Bq=[b11 b12 b13;b21 b22 b23;b31 b32 b33];

    %Coriolis Matrix
    c11=-c*(2*x(7)+x(8))*x(8)*sin(x(5))+f1*x(7);
    c12=c*(x(7)^2)*sin(x(5))+f2*x(8);
    c13=f3*x(9);
    Cq=[c11;c12;c13];

    %Gravity;
    g1=0;
    g2=0;
    g3=-e;
    Gq=[g1;g2;g3];

    %PID control
    %Parameters for thetha 1
    Kp1=Kpid(1);
    Kd1=Kpid(2);
    Ki1=Kpid(3);
    %Parameters for theta 2
    %Kp2=Kpid(2);
    Kp2=Kpid(4);
    Kd2=Kpid(5);
    Ki2=Kpid(6);
    %Parameters for d3
    %Kp3=Kpid(3);
    Kp3=Kpid(7);
    Kd3=Kpid(8);
    Ki3=Kpid(9);
    %Decoupled control input 
    %f1=Kp1*(th1s-x(4));
    f1=Kp1*(th1s-x(4))%-Kd1*x(7)+Ki1*(x(1));
    %f2=Kp2*(th1s-x(5));
    f2=Kp2*(th2s-x(5))%-Kd2*x(8)+Ki2*(x(2));
    %f1=Kp1*(th1s-x(4));
    f3=Kp3*(th3s-x(6))%-Kd3*x(9)+Ki3*(x(3));
    Fhat=[f1;f2;f3];

    F=Bq*Fhat;%actual input to the system

    %System states
    xdot(1)=th1s-x(4);
    xdot(2)=th2s-x(5);
    xdot(3)=th3s-x(6);

    xdot(4)=x(7);
    xdot(5)=x(8);
    xdot(6)=x(9);

    q2dot=inv(Bq)*(-Cq-Gq+F);
    xdot(7)=q2dot(1);
    xdot(8)=q2dot(2);
    xdot(9)=q2dot(3);
    
    %control input function output to outside computer program
    xdot(10)=F(1);
    xdot(11)=F(2);
    xdot(12)=F(3);

