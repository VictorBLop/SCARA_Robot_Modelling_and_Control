%function [B1 B2 B3 B4 BM Q1 Q2 Q3 Q4 Q5 Q6 Q7 Q8 QM n alpha] = inicia_dinam(in)
kp=100;
kd=100;
grav=9.81;

q1_sym=sym('q1');q2_sym=sym('q2');q3_sym=sym('q3');q4_sym=sym('q4');
q_sym=[q1_sym q2_sym q3_sym q4_sym];
q1d_sym=sym('q1d');q2d_sym=sym('q2d');q3d_sym=sym('q3d');q4d_sym=sym('q4d');
qd_sym=[q1d_sym q2d_sym q3d_sym q4d_sym];

d0=1;a1=0.5;a2=0.5;l1=0.25;l2=0.25;
mc=3;
m1=25;m2=25;m3=10;m4=0+mc;
mm1=0;mm2=0;mm3=0;mm4=0;
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

I1=[0 0 0;0 0 0;0 0 Il1+m1*(a1/2)^2+IM2];
I2=[0 0 0;0 0 0;0 0 Il2+m2*(a2/2)^2-IM3];
I3=[0 0 0;0 0 0;0 0 0];
I4=[0 0 0;0 0 0;0 0 Il4+m4*(l2/2)^2];
Im1=[0 0 0;0 0 0;0 0 IM1];
Im2=[0 0 0;0 0 0;0 0 IM2];
Im3=[0 0 0;0 0 0;0 0 IM3];
Im4=[0 0 0;0 0 0; 0 0 IM4];

B=simplify(m1*Jp1_t*Jp1+Jo1_t*R1*I1*R1_t*Jo1+mm1*Jpm1_t*Jpm1+Jom1_t*Rm1*Im1*Rm1_t*Jom1);
B=simplify(B+m2*Jp2_t*Jp2+Jo2_t*R2*I2*R2_t*Jo2+mm2*Jpm2_t*Jpm2+Jom2_t*Rm2*Im2*Rm2_t*Jom2);
B=simplify(B+m3*Jp3_t*Jp3+Jo3_t*R3*I3*R3_t*Jo3+mm3*Jpm3_t*Jpm3+Jom3_t*Rm3*Im3*Rm3_t*Jom3);
B=simplify(B+m4*Jp4_t*Jp4+Jo4_t*R4*I4*R4_t*Jo4+mm4*Jpm4_t*Jpm4+Jom4_t*Rm4*Im4*Rm4_t*Jom4);
B1=B(:,1);B2=B(:,2);B3=B(:,3);B4=B(:,4);
B_vector=[B1;B2;B3;B4];

%Calculo de C(q,q^.)
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
%C1=C(:,1);C2=C(:,2);C3=C(:,3);C4=C(:,4);

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
%Fv1=Fv(:,1);Fv2=Fv(:,2);Fv3=Fv(:,3);Fv4=Fv(:,4);

n=simplify(C*qd_sym'+Fv*qd_sym'+G);

Kp=kp*eye(4);
Kd=kd*eye(4);
K=[Kp Kd];
qM=[7*pi/8 7*pi/8 .4 7*pi/8];q1=qM(1);q2=qM(2);q3=qM(3);q4=qM(4);
BM=norm(eval(B));
qm=-qM;q1=qm(1);q2=qm(2);q3=qm(3);q4=qm(4);
Bm=norm(eval(B));
alpha=(BM-Bm)/(BM+Bm);
Q12=inv(eye(4)-Kp);
Q11=inv(eye(4)-inv(Kp))*(Kd*Q12-inv(Kp)*Q12*Kd*Kp)*inv(eye(4)-Kp);
Q22=inv(Kp)*(Q11-Q12*Kd);
Q=[Q11 Q12;Q12 Q22];
Q1=Q(:,1);Q2=Q(:,2);Q3=Q(:,3);Q4=Q(:,4);Q5=Q(:,5);Q6=Q(:,6);Q7=Q(:,7);Q8=Q(:,8);
Q_vector=[Q1;Q2;Q3;Q4;Q5;Q6;Q7;Q8];
qM=[7*pi/8 7*pi/8 .4 7*pi/8];q1=qM(1);q2=qM(2);q3=qM(3);q4=qM(4);
%QM=norm(eval(Q));
QM=norm(Q);

