%%Robot Siciliano
%Elaborato 2018/19

%Parámetros
%a1=0.5; a2=a1;
%l1=0.25; l2=l1;
syms a1 a2 l1 l2 ; 
syms q1 q2 q3 q4;
q=[q1 q2 q3 q4];
pi_s=sym('pi');

%D-H
A(1)=Link([0 0 a1 0 0]);
A(2)=Link([0 0 a2 pi 0]);
A(3)=Link([0 0 0 0 1]);
A(3).offset=l1;
A(4)=Link([0 l2 0 0 0]);
robot=SerialLink(A);
 
%Límites
%robot.qlim(1,1)=0; robot.qlim(1,2)=2*pi;
%robot.qlim(2,1)=-pi/2; robot.qlim(2,2)=pi/2;
%robot.qlim(3,1)=0; robot.qlim(3,2)=0.5;
%robot.qlim(4,1)=-pi/2; robot.qlim(4,2)=pi/2;

%Mostramos el robot
%robot.teach()
%q=[pi/4 pi/4 0.1 0];
T=double(simplify(robot.fkine(q))); %crea la matriz de transformación homogénea
%robot.plot(q);
Px=T(1,4);
Py=T(2,4);
Pz=T(3,4);

syms ec1 ec2 ec3
ec1=Px;
ec2=Py;
ec3=Pz;


Rot=[T_sym(1,1:3); T_sym(2,1:3); T_sym(3,1:3)];
w=Rot*eu;
syms q1 q2 q3 q4


J_a =   [diff(T_sym(1,4),q1), diff(T_sym(1,4),q2), diff(T_sym(1,4),q3), diff(T_sym(1,4),q4);
         diff(T_sym(2,4),q1), diff(T_sym(2,4),q2), diff(T_sym(2,4),q3), diff(T_sym(2,4),q4);
         diff(T_sym(3,4),q1), diff(T_sym(3,4),q2), diff(T_sym(3,4),q3), diff(T_sym(3,4),q4)];
     
Jacob_t=[ - a2*sin(q1 + q2) - a1*sin(q1), a2*cos(q1 + q2) + a1*cos(q1),0;
           -a2*sin(q1 + q2),                            a2*cos(q1+ q2),0;
                          0,                                         0,-1;
                          0,                                         0,0];

%Cinemática Inversa                  
cos2=(Px^2+Py^2-a1^2-a2^2)/(2*a1*a2);
sin2=sqrt(1-cos2^2);
Q2=simplify(atan2(sin2,cos2));  %Q2 mediante inversión cinemática

sin1=simplify(((a1+a2*cos2)*Py-a2*sin2*Px)/(Px^2+Py^2));
cos1=simplify(((a1+a2*cos2)*Px+a2*sin2*Py)/(Px^2+Py^2));
Q1=simplify(atan2(sin1,cos1));  %Q1 mediante inversión cinemática

Q3=-l1-l2-Pz;   %Q3 mediante inversión cinemática
%%
%Elipsoides de Manipulabilidad   
[V,D]=eig(Jacob*Jacob_t); %Autovalores y autovectores 
V=simplify(V);
D=simplify(D);

%Parámetros
a1=0.5; a2=a1;
l1=0.25; l2=l1;
pi_s=pi;
q1=pi/4;
q2=-pi/2;
q3=0.1;
q4=0;
d0=1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Elipsoides de velocidad %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[x,y,z]=ellipsoid(0, 0, 0, eval(D(1,1)), eval(D(2,2)), eval(D(3,3)), 20);
R=eval(V);
%Dibujamos el robot
A_vel=[];
i=1;
while(i<20)
    A_vel=[A_vel;(R*[x(:,i)';y(:,i)';z(:,i)'])'];
    i=i+1;
end
figure(2)
plot3(A_vel(:,1)+eval(T(1,4)),A_vel(:,2)+eval(T(2,4)),A_vel(:,3)+eval(T(3,4))+d0);
grid on
hold on
plot3([0 0], [0 0], [0 1])
plot3([0 a1*cos(q1)], [0 a1*sin(q1)], [1 1]);
plot3([a1*cos(q1) a1*cos(q1)+a2*cos(q1+q2)], [a1*sin(q1) a1*sin(q1)+a2*sin(q1+q2)], [1 1]);
plot3([a1*cos(q1)+a2*cos(q1+q2) a1*cos(q1)+a2*cos(q1+q2)], [a1*sin(q1)+a2*sin(q1+q2) a1*sin(q1)+a2*sin(q1+q2)], [1 0.5-q3]);
%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Elipsoides de fuerza %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[x1,y1,z1]=ellipsoid(0, 0, 0, eval(D(2,2)), eval(D(1,1)), eval(D(3,3)), 20);
%Los autovectores son los mismos, sin embargo los autovalores son
%diferentes. Son recíprocos. Por ello, los cambiamos de eje x a y y
%viceversa.
R=eval(V);
%Dibujamos el robot
A_force=[];
i=1;
%R90=[0 -1 0; 1 0 0; 0 0 1];
while(i<20)
    A_force=[A_force;(R*[x1(:,i)';y1(:,i)';z1(:,i)'])'];
    i=i+1;
end
figure(2)
plot3(A_force(:,1)+eval(T(1,4)),A_force(:,2)+eval(T(2,4)),A_force(:,3)+eval(T(3,4))+d0,'r');
grid on
hold on
plot3([0 0], [0 0], [0 1])
plot3([0 a1*cos(q1)], [0 a1*sin(q1)], [1 1]);
plot3([a1*cos(q1) a1*cos(q1)+a2*cos(q1+q2)], [a1*sin(q1) a1*sin(q1)+a2*sin(q1+q2)], [1 1]);
plot3([a1*cos(q1)+a2*cos(q1+q2) a1*cos(q1)+a2*cos(q1+q2)], [a1*sin(q1)+a2*sin(q1+q2) a1*sin(q1)+a2*sin(q1+q2)], [1 0.5-q3]);

