%% Progetto Controllo dei Robot (Napoli, 2018/19) %%
format short g
pi_sym=sym('pi');

%% Parámetros geométricos del brazo %%
d0=1;a1=0.5;a2=0.5;l1=0.25;l2=0.25;
d0_sym=sym('d0');a1_sym=sym('a1');a2_sym=sym('a2');l1_sym=sym('l1');l2_sym=sym('l2');

% Coeficientes D-H %
L_sym(1)=Link([0 0 a1_sym 0 0]);
L_sym(2)=Link([0 0 a2_sym pi_sym 0]);
L_sym(3)=Link([0 0 0 0 1]);   % Articulación auxiliar (no real)
L_sym(3).offset=l1_sym;
L_sym(4)=Link([0 l2_sym 0 0 0]);
scara_sym=SerialLink(L_sym);

L(1)=Link([0 0 a1 0 0]);
L(2)=Link([0 0 a2 pi 0]);
L(3)=Link([0 0 0 0 1]);   % Articulación auxiliar (no real)
L(3).offset=l1;
L(4)=Link([0 l2 0 0 0]);
scara=SerialLink(L);

%% Prueba visual %%
scara.qlim(1,1:2)=[-pi/2 pi/2];
scara.qlim(2,1:2)=[-pi/2 pi/2];
scara.qlim(3,1:2)=[0 d0-l1-l2];
scara.qlim(4,1:2)=[-pi/2 pi/2];
% scara.teach();

%% Matrices de transformación %%
q1_sym=sym('q1');q2_sym=sym('q2');q3_sym=sym('q3');q4_sym=sym('q4');
q_sym=[q1_sym q2_sym q3_sym q4_sym];

art1=SerialLink(L_sym(1));
% 'A01 ='
A01_sym=simplify(art1.fkine(q1_sym));

art2=SerialLink(L_sym(2));
% 'A12 ='
A12_sym=simplify(art2.fkine(q2_sym));
art12=SerialLink(L_sym(1:2));
% 'A02 ='
A02_sym=(simplify(art12.fkine(q_sym(1:2))));

art3=SerialLink(L_sym(3));
% 'A23 ='
A23_sym=simplify(art3.fkine(q3_sym));
art123=SerialLink(L_sym(1:3));
% 'A03 ='
A03_sym=(simplify(art123.fkine(q_sym(1:3))));
 
art4=SerialLink(L_sym(4));
% 'A34 ='
A34_sym=(simplify(art4.fkine(q4_sym)));

%% Matriz T del modelo cinemático directo %%
% 'T ='
T_sym=(simplify(scara_sym.fkine(q_sym)));

%% Método cinemático inverso %%
px_sym=sym('px');py_sym=sym('py');pz_sym=sym('pz');

cos2=(px_sym^2+py_sym^2-a1^2-a2^2)/(2*a1*a2);
sin2=sqrt(1-cos2^2);
q2_fun=simplify(atan2(sin2,cos2));

sin1=simplify(((a1+a2*cos2)*py_sym-a2*sin2*px_sym)/(px_sym^2+py_sym^2));
cos1=simplify(((a1+a2*cos2)*px_sym+a2*sin2*py_sym)/(px_sym^2+py_sym^2));
q1_fun=simplify(atan2(sin1,cos1));

q3_fun=-l1-l2-pz_sym;

%% Jacobiano %%
z=[0;0;1];
pe_sym=[a2*cos(q1_sym + q2_sym) + a1*cos(q1_sym); a2*sin(q1_sym + q2_sym) + a1*sin(q1_sym); d0 - l1 - l2 - q3_sym];

Jp1_sym=cross(z, pe_sym-[0;0;0]);
Jp2_sym=cross(z, pe_sym-[a1*cos(q1_sym); a1*sin(q1_sym); d0]);
Jp3_sym=-z;
Jp4_sym=cross(-z, pe_sym-[a2*cos(q1_sym + q2_sym) + a1*cos(q1_sym); a2*sin(q1_sym + q2_sym) + a1*sin(q1_sym); d0 - l1 - q3_sym]);
Jp_sym=[Jp1_sym Jp2_sym Jp3_sym Jp4_sym];

Jo1_sym=z;
Jo2_sym=z;
Jo3_sym=[0;0;0];
Jo4_sym=-z;
Jo_sym=[Jo1_sym Jo2_sym Jo3_sym Jo4_sym];

J_sym=[Jp_sym; Jo_sym];

Px=T_sym(1,4);
Py=T_sym(2,4);
Pz=T_sym(3,4);

%% Elipsoides %%
Jp_sym_t=[- a2*sin(q1_sym + q2_sym) - a1*sin(q1_sym), a2*cos(q1_sym + q2_sym) + a1*cos(q1_sym), 0
    -a2*sin(q1_sym + q2_sym), a2*cos(q1_sym + q2_sym), 0
    0, 0, -1
    0, 0, 0];

[V_sym D_sym]=eig(Jp_sym*Jp_sym_t);
V_sym=simplify(V_sym);D_sym=simplify(D_sym);

%Introducimos los valores de las articulaciones:
%Para las elipsoides de manipulabilidad:
%Q =-1.4706    2.9413   -1.0000
%   -1.3694    2.7389   -1.0000
%   -1.2661    2.5322   -1.0000
%   -1.1593    2.3186   -1.0000
%   -1.0472    2.0944   -1.0000
%   -0.9273    1.8546   -1.0000
%   -0.7954    1.5908   -1.0000
%   -0.6435    1.2870   -1.0000   
%   -0.4510    0.9021   -1.0000
%      0         0   -1.0000
q1=-0.4510; q2=0.9021; q3=-1; q4=0;

e=[];
for i=[0,d0]
    e=[e;0 0 i];
end
e=[e;eval(A01_sym(1,4)) eval(A01_sym(2,4)) eval(A01_sym(3,4))+d0];
e=[e;eval(A02_sym(1,4)) eval(A02_sym(2,4)) eval(A02_sym(3,4))+d0];
e=[e;eval(A03_sym(1,4)) eval(A03_sym(2,4)) eval(A03_sym(3,4))+d0];
e=[e;eval(T_sym(1,4)) eval(T_sym(2,4)) eval(T_sym(3,4))+d0];


R=eval(V_sym);
N=20;
[x y z]=ellipsoid(0,0,0,double(eval(D_sym(1,1))),double(eval(D_sym(2,2))),double(eval(D_sym(3,3))),N);

A=[];

% X=[];Y=[];Z=[];
%Elipsoide de velocidad
for i=[1:N+1]
    A=[A;(R*[x(:,i)';y(:,i)';z(:,i)'])'];
    %Para cambiar el dibujo de la elipsoide
%     A=[A;(R*[x(i,:);y(i,:);z(i,:)])'];  
end

%Elipsoide de fuerza
[x1 y1 z1]=ellipsoid(0,0,0,double(eval(D_sym(2,2))),double(eval(D_sym(1,1))),double(eval(D_sym(3,3))),N);
A_f=[];
for i=[1:N+1]
    A_f=[A_f;(R*[x1(:,i)';y1(:,i)';z1(:,i)'])'];
    %Para cambiar el dibujo de la elipsoide
%     A=[A;(R*[x(i,:);y(i,:);z(i,:)])'];  
end
% for i=[1:N+1]
%     for j=[1:N+1]
%         X(i,j)=A(j+(i-1)*N,1);
%         Y(i,j)=A(j+(i-1)*N,2);
%         Z(i,j)=A(j+(i-1)*N,3);
%     end
% end
%plot3(e(:,1),e(:,2),e(:,3));

figure(1)
plot3(A(:,1)+eval(T_sym(1,4)),A(:,2)+eval(T_sym(2,4)),A(:,3)+eval(T_sym(3,4)),'b'); 
hold on
grid on
plot3(A_f(:,1)+eval(T_sym(1,4)),A_f(:,2)+eval(T_sym(2,4)),A_f(:,3)+eval(T_sym(3,4)),'r');
title('Ellissoidi di manipolabilità');
plot3([0 0], [0 0], [0 1],'black')
plot3([0 a1*cos(q1)], [0 a1*sin(q1)], [1 1],'black');
plot3([a1*cos(q1) a1*cos(q1)+a2*cos(q1+q2)], [a1*sin(q1) a1*sin(q1)+a2*sin(q1+q2)], [1 1],'black');
plot3([a1*cos(q1)+a2*cos(q1+q2) a1*cos(q1)+a2*cos(q1+q2)], [a1*sin(q1)+a2*sin(q1+q2) a1*sin(q1)+a2*sin(q1+q2)], [1 -0.5-q3], 'black');
xlabel('Asse X (m)'); ylabel('Asse Y (m)'); zlabel('Asse Z (m)');
legend('Ellissoide de velocità','Ellissoide de forza', 'Robot');

figure(2)
subplot(1,2,1);
plot(A(:,1)+eval(T_sym(1,4)),A(:,2)+eval(T_sym(2,4)),'b');
hold on; grid on
plot(A_f(:,1)+eval(T_sym(1,4)),A_f(:,2)+eval(T_sym(2,4)),'r');
plot([0 0], [0 0],'black')
plot([0 a1*cos(q1)], [0 a1*sin(q1)],'black');
plot([a1*cos(q1) a1*cos(q1)+a2*cos(q1+q2)], [a1*sin(q1) a1*sin(q1)+a2*sin(q1+q2)],'black');
plot([a1*cos(q1)+a2*cos(q1+q2) a1*cos(q1)+a2*cos(q1+q2)], [a1*sin(q1)+a2*sin(q1+q2) a1*sin(q1)+a2*sin(q1+q2)],'black');
xlabel('Asse X (m)'); ylabel('Asse Y (m)');
legend('Ellissoide de velocità','Ellissoide de forza', 'Robot');

subplot(1,2,2);
plot(A(:,2)+eval(T_sym(2,4)),A(:,3)+eval(T_sym(3,4)),'b'); 
hold on; grid on
plot(A_f(:,2)+eval(T_sym(2,4)),A_f(:,3)+eval(T_sym(3,4)),'r');
plot([0 0], [0 1],'black')
plot( [0 a1*sin(q1)], [1 1],'black');
plot( [a1*sin(q1) a1*sin(q1)+a2*sin(q1+q2)], [1 1],'black');
plot([a1*sin(q1)+a2*sin(q1+q2) a1*sin(q1)+a2*sin(q1+q2)], [1 -0.5-q3], 'black');
xlabel('Asse Y (m)'); ylabel('Asse Z (m)'); zlabel('Asse Z (m)');
legend('Ellissoide de velocità','Ellissoide de forza', 'Robot');
%% Cinematica Inversa Jacobiana Analítica %%
%Jacobiana Inversa
%Jacobiana Analítica
syms q11 q22 q33 q44
Rot=[T_sym(1,1:3); T_sym(2,1:3); T_sym(3,1:3)];
w=[0;0;q11+q22-q44];
syms eux euy euz
eu=Rot'*w;
eux=eu(1); euy=eu(2); euz=eu(3);

J_a=[J_sym(1,:); J_sym(2,:); J_sym(3,:);[0 0 0 0];[0 0 0 0];[-1 -1 0 1]];
%Eliminamos las filas que son 0 para que sea invertible.
J_a_red=[J_a(1,:);J_a(2,:);J_a(3,:);J_a(6,:)];

tras_J_a=[J_a_red(1,1) J_a_red(2,1) J_a_red(3,1) J_a_red(4,1);
          J_a_red(1,2) J_a_red(2,2) J_a_red(3,2) J_a_red(4,2);
          J_a_red(1,3) J_a_red(2,3) J_a_red(3,3) J_a_red(4,3);
          J_a_red(1,4) J_a_red(2,4) J_a_red(3,4) J_a_red(4,4)];
      
inv_J_a=simplify(inv(J_a_red));

K=eye(4)*100;
dif_t=1e-3;
xi=0.5;
yi=0.2;
zi=0.3;
phii=0;
xf=0.3;
yf=0.3;
zf=0.5;
phif=1.7;
durata=4;
N=8;
%Abbiamo 3 variable per usare: q (pos. articolare), qd (velocità
%articolare) e xe_1 (posizione).

%% MODELO DINAMICO %%
%Matriz B
mc=3;
m1=25;m2=25;m3=10;m4=0+mc;
mm1=10;mm2=10;mm3=10;mm4=10;
kr1=1;kr2=1;kr3=50;kr4=20;
Il1=5;Il2=5;Il3=0;Il4=1;
IM1=.02;IM2=.02;IM3=.005;IM4=.001;
Fm1=.0001;Fm2=.0001;Fm3=.01;Fm4=.005;

Jp1=[-l1/2*sin(q1_sym) 0 0 0;l1/2*cos(q1_sym) 0 0 0;0 0 0 0];
Jp1_t=[-l1/2*sin(q1_sym) -l1/2*cos(q1_sym) 0;0 0 0;0 0 0;0 0 0];
Jp2=[-l1*sin(q1_sym)-l2/2*sin(q1_sym+q2_sym) -l2/2*sin(q1_sym+q2_sym) 0 0;l1*cos(q1_sym)+l2/2*cos(q1_sym+q2_sym) l2/2*sin(q1_sym+q2_sym) 0 0;0 0 0 0];
Jp2_t=[-l1*sin(q1_sym)-l2/2*sin(q1_sym+q2_sym) l1*cos(q1_sym)+l2/2*cos(q1_sym+q2_sym) 0;-l2/2*sin(q1_sym+q2_sym) l2/2*sin(q1_sym+q2_sym) 0;0 0 0;0 0 0];
Jp3=[-l1*sin(q1_sym)-l2*sin(q1_sym+q2_sym) -l2*sin(q1_sym+q2_sym) 0 0;l1*cos(q1_sym)+l2*cos(q1_sym+q2_sym) l2*sin(q1_sym+q2_sym) 0 0;0 0 -1 0];
Jp3_t=[-l1*sin(q1_sym)-l2*sin(q1_sym+q2_sym) l1*cos(q1_sym)+l2*cos(q1_sym+q2_sym) 0;-l2*sin(q1_sym+q2_sym) l2*sin(q1_sym+q2_sym) -1;0 0 0;0 0 0];
Jp4=Jp3;
Jp4_t=Jp3_t;
Jo1=[0 0 0 0;0 0 0 0;1 0 0 0];
Jo1_t=[0 0 1;0 0 0;0 0 0;0 0 0];
Jo2=[0 0 0 0;0 0 0 0;1 1 0 0];
Jo2_t=[0 0 1;0 0 1;0 0 0;0 0 0];
Jo3=Jo2;
Jo3_t=Jo2_t;
Jo4=[0 0 0 0;0 0 0 0;1 1 0 -1];
Jo4_t=[0 0 1;0 0 1;0 0 0;0 0 -1];
Jpm1=[0 0 0 0;0 0 0 0;0 0 0 0];
Jpm1_t=[0 0 0;0 0 0;0 0 0;0 0 0];
Jpm2=[-l1*sin(q1_sym) 0 0 0;l1*cos(q1_sym) 0 0 0;0 0 0 0];
Jpm2_t=[-l1*sin(q1_sym) l1*cos(q1_sym) 0;0 0 0;0 0 0;0 0 0];
Jpm3=[-l1*sin(q1_sym)-l2*sin(q1_sym+q2_sym) -l2*sin(q1_sym+q2_sym) 0 0;l1*cos(q1_sym)+l2*cos(q1_sym+q2_sym) l2*cos(q1_sym+q2_sym) 0 0;0 0 0 0];
Jpm3_t=[-l1*sin(q1_sym)-l2*sin(q1_sym+q2_sym) l1*cos(q1_sym)+l2*cos(q1_sym+q2_sym) 0;-l2*sin(q1_sym+q2_sym) l2*cos(q1_sym+q2_sym) 0;0 0 0;0 0 0];
Jpm4=[-l1*sin(q1_sym)-l2*sin(q1_sym+q2_sym) -l2*sin(q1_sym+q2_sym) 0 0;l1*cos(q1_sym)+l2*cos(q1_sym+q2_sym) l2*cos(q1_sym+q2_sym) 0 0;0 0 -1 0];
Jpm4_t=[-l1*sin(q1_sym)-l2*sin(q1_sym+q2_sym) l1*cos(q1_sym)+l2*cos(q1_sym+q2_sym) 0;-l2*sin(q1_sym+q2_sym) l2*cos(q1_sym+q2_sym) -1;0 0 0;0 0 0];
Jom1=[0 0 0 0;0 0 0 0;kr1 0 0 0];
Jom1_t=[0 0 kr1;0 0 0;0 0 0;0 0 0];
Jom2=[0 0 0 0;0 0 0 0;1 kr2 0 0];
Jom2_t=[0 0 1;0 0 kr2;0 0 0;0 0 0];
Jom3=[0 0 0 0;0 0 0 0;1 1 -kr3 0];
Jom3_t=[0 0 1;0 0 1;0 0 -kr3;0 0 0];
Jom4=[0 0 0 0;0 0 0 0;1 1 0 -kr4];
Jom4_t=[0 0 1;0 0 1;0 0 0;0 0 -kr4];

R1=[cos(q1_sym) -sin(q1_sym) 0;sin(q1_sym) -cos(q1_sym) 0;0 0 1];
R1_t=[cos(q1_sym) sin(q1_sym) 0;-sin(q1_sym) -cos(q1_sym) 0;0 0 1];
R2=[cos(q1_sym+q2_sym) -sin(q1_sym+q2_sym) 0;sin(q1_sym+q2_sym) -cos(q1_sym+q2_sym) 0;0 0 -1];
R2_t=[cos(q1_sym+q2_sym) sin(q1_sym+q2_sym) 0;-sin(q1_sym+q2_sym) -cos(q1_sym+q2_sym) 0;0 0 -1];
R3=R2;
R3_t=R2_t;
R4=[cos(q1_sym+q2_sym-q4_sym) -sin(q1_sym+q2_sym-q4_sym) 0;sin(q1_sym+q2_sym-q4_sym) -cos(q1_sym+q2_sym-q4_sym) 0;0 0 -1];
R4_t=[cos(q1_sym+q2_sym-q4_sym) sin(q1_sym+q2_sym-q4_sym) 0;-sin(q1_sym+q2_sym-q4_sym) -cos(q1_sym+q2_sym-q4_sym) 0;0 0 -1];
Rm1=[cos(kr1*q1_sym) -sin(kr1*q1_sym) 0;sin(kr1*q1_sym) -cos(kr1*q1_sym) 0;0 0 1];
Rm1_t=[cos(kr1*q1_sym) sin(kr1*q1_sym) 0;-sin(kr1*q1_sym) -cos(kr1*q1_sym) 0;0 0 1];
Rm2=[cos(q1_sym+kr2*q2_sym) -sin(q1_sym+kr2*q2_sym) 0;sin(q1_sym+kr2*q2_sym) -cos(q1_sym+kr2*q2_sym) 0;0 0 -1];
Rm2_t=[cos(q1_sym+kr2*q2_sym) sin(q1_sym+kr2*q2_sym) 0;-sin(q1_sym+kr2*q2_sym) -cos(q1_sym+kr2*q2_sym) 0;0 0 -1];
Rm3=[cos(q1_sym+q2_sym-kr3*q3_sym) -sin(q1_sym+q2_sym-kr3*q3_sym) 0;sin(q1_sym+q2_sym-kr3*q3_sym) -cos(q1_sym+q2_sym-kr3*q3_sym) 0;0 0 -1];
Rm3_t=[cos(q1_sym+q2_sym-kr3*q3_sym) sin(q1_sym+q2_sym-kr3*q3_sym) 0;-sin(q1_sym+q2_sym-kr3*q3_sym) -cos(q1_sym+q2_sym-kr3*q3_sym) 0;0 0 -1];
Rm4=[cos(q1_sym+q2_sym-kr4*q4_sym) -sin(q1_sym+q2_sym-kr4*q4_sym) 0;sin(q1_sym+q2_sym-kr4*q4_sym) -cos(q1_sym+q2_sym-kr4*q4_sym) 0;0 0 -1];
Rm4_t=[cos(q1_sym+q2_sym-kr4*q4_sym) sin(q1_sym+q2_sym-kr4*q4_sym) 0;-sin(q1_sym+q2_sym-kr4*q4_sym) -cos(q1_sym+q2_sym-kr4*q4_sym) 0;0 0 -1];

I1=[0 0 0;0 0 0;0 0 Il1];
I2=[0 0 0;0 0 0;0 0 Il2];
I3=[0 0 0;0 0 0;0 0 -Il3];
I4=[0 0 0;0 0 0;0 0 -Il4];
Im1=[0 0 0;0 0 0;0 0 IM1];
Im2=[0 0 0;0 0 0;0 0 IM2];
Im3=[0 0 0;0 0 0;0 0 IM3];
Im4=[0 0 0;0 0 0; 0 0 -IM4];

B=simplify(m1*Jp1_t*Jp1+Jo1_t*R1*I1*R1_t*Jo1+mm1*Jpm1_t*Jpm1+Jom1_t*Rm1*Im1*Rm1_t*Jom1);
B=simplify(B+m2*Jp2_t*Jp2+Jo2_t*R2*I2*R2_t*Jo2+mm2*Jpm2_t*Jpm2+Jom2_t*Rm2*Im2*Rm2_t*Jom2);
B=simplify(B+m3*Jp3_t*Jp3+Jo3_t*R3*I3*R3_t*Jo3+mm3*Jpm3_t*Jpm3+Jom3_t*Rm3*Im3*Rm3_t*Jom3);
B=simplify(B+m4*Jp4_t*Jp4+Jo4_t*R4*I4*R4_t*Jo4+mm4*Jpm4_t*Jpm4+Jom4_t*Rm4*Im4*Rm4_t*Jom4)

%Calculo de C(q,q^.)
q1d_sym=sym('q1d');q2d_sym=sym('q2d');q3d_sym=sym('q3d');q4d_sym=sym('q4d');
qd_sym=[q1d_sym q2d_sym q3d_sym q4d_sym];
syms c1 c2 c3 c4
for j=1:4
    for k=1:4
        c1(j,k)=(diff(B(1,j),q_sym(k))+diff(B(1,k),q_sym(j))-diff(B(j,k),q_sym(1)))/2;
        c2(j,k)=(diff(B(2,j),q_sym(k))+diff(B(2,k),q_sym(j))-diff(B(j,k),q_sym(2)))/2;
        c3(j,k)=(diff(B(3,j),q_sym(k))+diff(B(3,k),q_sym(j))-diff(B(j,k),q_sym(3)))/2;
        c4(j,k)=(diff(B(4,j),q_sym(k))+diff(B(4,k),q_sym(j))-diff(B(j,k),q_sym(4)))/2;
    end
end
c1=simplify(c1);c2=simplify(c2);c3=simplify(c3);c4=simplify(c4);
syms c
c=sym('c',[4 4]);

for k=1:4
    for j=1:4
        for i=1:4
            if(i==1)
                c(i,j)=c(i,j)+c1(j,k)*qd_sym(k);
            elseif(i==2)
                c(i,j)=c(i,j)+c2(j,k)*qd_sym(k);
            elseif(i==3)
                c(i,j)=c(i,j)+c3(j,k)*qd_sym(k);    
            else
                c(i,j)=c(i,j)+c4(j,k)*qd_sym(k); 
            end
        end
    end
end

c1_1=0; c1_2=0; c1_3=0; c1_4=0;
c2_1=0; c2_2=0; c2_3=0; c2_4=0;
c3_1=0; c3_2=0; c3_3=0; c3_4=0;
c4_1=0; c4_2=0; c4_3=0; c4_4=0;

C=simplify(c); %Coficiente C (centrífugo y de Coriolis)

%Cálculo valor de términos gravitatorios
syms g grav
go=[0 0 -grav];
g=sym('g',[4 1]);
mL=[m1 m2 m3 m4];
mm=[mm1 mm2 mm3 mm4];
for i=1:4
    for j=1:4
        if(j==1)
            g(i)=g(i)-(mL(j)*go*Jp1(:,i)+mm(j)*go*Jpm1(:,i));
        elseif(j==2)
            g(i)=g(i)-(mL(j)*go*Jp2(:,i)+mm(j)*go*Jpm2(:,i));
        elseif(j==3)
            g(i)=g(i)-(mL(j)*go*Jp3(:,i)+mm(j)*go*Jpm3(:,i));
        else
            g(i)=g(i)-(mL(j)*go*Jp4(:,i)+mm(j)*go*Jpm4(:,i));
        end
    end
end
g1=0; g2=0; g3=0; g4=0;
G=simplify(g); % G -> gravedad

%Fuerzas viscosas (forze viscose)
%f_ii=kri^2*f_mii (f_mii = Fm (dado en el enunciado))
f11=Fm1*kr1^2;
f22=Fm2*kr2^2;
f33=Fm3*kr3^2;
f44=Fm4*kr4^2;
Fv=[f11 0 0 0; 0 f22 0 0; 0 0 f33 0; 0 0 0 f44];%Fv

%% CONTROLLO DINAMICO %%
Kp=100*eye(4);
Kd=50*eye(4);

%Derivada de la Jacobiana analítica
syms J_a_d
J_a_d(1,1)= -cos(q1_sym+q2_sym)*q1d_sym/2 -cos(q1_sym)*q1d_sym/2;
J_a_d(1,2)= -cos(q1_sym)*q1d_sym/2;
J_a_d(1,3)=0; J_a_d(1,4)=0;
J_a_d(2,1)= -sin(q1_sym+q2_sym)*q1d_sym/2 -sin(q1_sym)*q1d_sym/2;
J_a_d(2,2)= -sin(q1_sym)*q1d_sym/2;
J_a_d(2,3)=0; J_a_d(2,4)=0;
J_a_d(3,1)=-cos(q1_sym+q2_sym)*q2d_sym/2;
J_a_d(3,2)=-cos(q1_sym+q2_sym)*q2d_sym/2;
J_a_d(3,3)=0; J_a_d(3,4)=0;
J_a_d(4,1)=-sin(q1_sym+q2_sym)*q2d_sym/2;
J_a_d(4,2)=-sin(q1_sym+q2_sym)*q2d_sym/2;
J_a_d(4,3)=0; J_a_d(4,4)=0;

              
                        
                    

    



