close all;clear all;clc;
s = tf('s');

R=1.9;  %Ohm
Ll=65*(10)^(-6);  %H
J=5.7*(10)^(-7); %kg.m2
b=0.000002;     %N.m.s
Ka=0.0134;

% x = [dtheta;i] 
A = [-b/J Ka/J  
     -Ka/Ll -R/Ll];  
B = [0;1/Ll]; 

%A = [1.0e+04*-0.0203 1.0e+04*-1.856;
%     1.0e+04*0.0001 0];
%B= [1.0e+04*1;0];
%C = [0 22670];

%A = [0 1.0e+04*0.0001;1.0e+04*-0.0203 1.0e+04*-1.856];
%B=[0;1.0e+04*1];

%A = [0              1.0e+04*0.0001;
%    1.0e+04*-0.0203 1.0e+04*-1.856];
%B=[0;1.0e+04*1];
C = [1 0];   
D = [0];

%A-B*K debe ser una matriz estable para aplicar LQR
%x=[x1;x2];u=-K*x;K=[k1 k2];
syms k1 k2; K=[k1 k2];
eig(A-B*K);
%Rpta: r1=-(4*k2)/5-((17*k2^2)/7+(6*k2)/7-(6*k1)/4+2)^(1/2)/2-4
%Rpta: r2=((17*k2^2)/7+(6*k2)/7-(6*k1)/4+2)^(1/2)/2-(4*k2)/5-4
%Como es posible que r1 y r2 tengan parte real negativa, entonces sí es
%estable. LQR se puede aplicar a este sistema.

%Determinar cuantos integreadores tiene el sistema
eig(A);%Rpta: -170;-29064 -> como no hay ceros, no tiene integradores

%Como la planta no tiene integreadores, integramos la salida
%Aumento de matrices por los no integradores
Aa=[A zeros(2,1);-C 0]; Ba=[B;0];


%Calculando controlabilidad (n=2)
rank(ctrb(Aa,Ba)); %Rpta. 3 -> completamente controlable

%Calculando observabilidad (n=2)
rank(obsv(A,C)); %Rpta. 2 -> completamente observable

%Función de costos: Q y R
Qa=1*[1 0 0
   0 1 0
   0 0 500];
R=[1];

%Ley de control: u=-Ke -> K=[Ka KI]
[Ka]=lqr(Aa,Ba,Qa,R); K=Ka(1:2);KI=-Ka(3);
%Ecuación de Riccati: A'*P+P*A-P*B*R(-1)*B'*P+Q=0
%[K,P,E]=lqr(A,B,Q,R);%P de Ricatti y E de eigenvalues

Qe=1*[1 0;0 1]; Re=[1];G=0.1*eye(2);
[L]=lqe(A,G,C,Qe,Re);
xe1=[0;0];u1=0;
xe2=[0;0];u2=0;
xe3=[0;0];u3=0;
xe4=[0;0];u4=0;

x1=[0;0];e1=0;N=45000;T=0.00001;
x2=[0;0];e2=0;
x3=[0;0];e3=0;
x4=[0;0];e4=0;
for k=1:N
    %Cinematica Directa
    vxd=2;VXd(k)=vxd;
    vyd=2;VYd(k)=vyd;
    thetad=1;THETAd(k)=thetad;
    [r1,r2,r3,r4]=cindir(vxd,vyd,thetad);
    %Referencia
    R1(k)=r1;
    R2(k)=r2;
    R3(k)=r3;
    R4(k)=r4;
    %Salida
    y1=C*x1;
    y2=C*x2;
    y3=C*x3;
    y4=C*x4;
    %Ecuación del integrador
    e1=e1+T*(r1-y1);
    e2=e2+T*(r2-y2);
    e3=e3+T*(r3-y3);
    e4=e4+T*(r4-y4);
    % Observador
    xe1=xe1+T*(A*xe1+B*u1+L*(y1-C*xe1));
    xe2=xe2+T*(A*xe2+B*u2+L*(y2-C*xe2));
    xe3=xe3+T*(A*xe3+B*u3+L*(y3-C*xe3));
    xe4=xe4+T*(A*xe4+B*u4+L*(y4-C*xe4));
    %Fuerza de control
    u1=-K*xe1+KI*e1;U1(k)=u1;
    u2=-K*xe2+KI*e2;U2(k)=u2;
    u3=-K*xe3+KI*e3;U3(k)=u3;
    u4=-K*xe4+KI*e4;U4(k)=u4;
    %Vector de estado
    x1=x1+T*(A*x1+B*u1); %Discretizando la planta
    x2=x2+T*(A*x2+B*u2);
    x3=x3+T*(A*x3+B*u3);
    x4=x4+T*(A*x4+B*u4);
    Y1(k)=x1(1);%Salida
    Y2(k)=x2(1);
    Y3(k)=x3(1);
    Y4(k)=x4(1);
    %Cinematica Indirecta
    [vx,vy,theta]=ci(x1(1),x2(1),x3(1),x4(1));
    VX(k)=vx;VY(k)=vy;THETA(k)=theta;
end

t=linspace(0,T*N,N);
subplot(3,1,1),plot(t,VXd,'r',t,VX,'b');ylabel('Velocidad (m/s)');grid;title('Velocidad en x');
axis([0 0.45 0 2.3])
legend('Velocidad en x deseada (m/s)','Velocidad en x (m/s)','Location','southeast'),xlabel('Tiempo (s)');

subplot(3,1,2),plot(t,VYd,'r',t,VY,'b');ylabel('Velocidad (m/s)');grid;title('Velocidad en y');
axis([0 0.45 0 2.3])
legend('Velocidad en y deseada (m/s)','Velocidad en y (m/s)','Location','southeast'),xlabel('Tiempo (s)');

subplot(3,1,3),plot(t,THETAd,'r',t,THETA,'b');ylabel('Velocidad (rad/s)');grid;title('Velocidad angular');
axis([0 0.45 0 1.1])
legend('Velocidad angular deseada (rad/s)','Velocidad angular (rad/s)','Location','southeast'),xlabel('Tiempo (s)');



figure,
subplot(4,1,1),plot(t,R1,'r',t,Y1,'b');ylabel('Velocidad (rad/s)');grid;title('Velocidad de la rueda 1');
axis([0 0.45 0 95])
legend('Velocidad angular deseada (rad/s)','Velocidad angular (rad/s)','Location','southeast'),xlabel('Tiempo (s)');

subplot(4,1,2),plot(t,R2,'r',t,Y2,'b');ylabel('Velocidad (rad/s)');grid;title('Velocidad de la rueda 2');
%axis([0 0.45 0 1.1])
legend('Velocidad angular deseada (rad/s)','Velocidad angular (rad/s)','Location','southeast'),xlabel('Tiempo (s)');

subplot(4,1,3),plot(t,R3,'r',t,Y3,'b');ylabel('Velocidad (rad/s)');grid;title('Velocidad de la rueda 3');
axis([0 0.45 0 40])
legend('Velocidad angular deseada (rad/s)','Velocidad angular (rad/s)','Location','southeast'),xlabel('Tiempo (s)');

subplot(4,1,4),plot(t,R4,'r',t,Y4,'b');ylabel('Velocidad (rad/s)');grid;title('Velocidad de la rueda 4');
%axis([0 0.45 0 1.1])
legend('Velocidad angular deseada (rad/s)','Velocidad angular (rad/s)','Location','southeast'),xlabel('Tiempo (s)');



figure,
subplot(4,1,1),plot(t,U1,'b');ylabel('Voltaje (V)');grid;title('Esfuerzo de control de la rueda 1');
axis([0 0.45 0 1.4])
xlabel('Tiempo (s)');

subplot(4,1,2),plot(t,U2,'b');ylabel('Voltaje (V)');grid;title('Esfuerzo de control de la rueda 2');
%axis([0 0.45 0 1.1])
xlabel('Tiempo (s)');

subplot(4,1,3),plot(t,U3,'b');ylabel('Voltaje (V)');grid;title('Esfuerzo de control de la rueda 3');
%axis([0 0.45 0 1.1])
xlabel('Tiempo (s)');

subplot(4,1,4),plot(t,U4,'b');ylabel('Voltaje (V)');grid;title('Esfuerzo de control de la rueda 4');
axis([0 0.45 0 1.4])
xlabel('Tiempo (s)');

%% Lazo cerrado
ALC=[A   -B*K          KI*B
    L*C   A-B*K-L*C    KI*B
    -C    0 0          0];
BLC=[0;0;0;0;1];
CLC=[C 0 0 0];
sysLC=ss(ALC,BLC,CLC,0);
step(sysLC)
ylabel('Velocidad (rad/s)');xlabel('Tiempo');title('Respuesta al escalón');
axis([0 0.45 0 1.1])
pzmap(sysLC)
ylabel('Eje imaginario');xlabel('Eje real');title('Mapa de polos y ceros');