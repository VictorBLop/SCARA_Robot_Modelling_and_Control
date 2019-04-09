%Transformacion de Jacobiana simbólica a numerica
function [qd]=JacobianaNumerica (in)
x1=in(1);
x2=in(2);
x3=in(3);
x4=in(4);
q1=in(5);
q2=in(6);
q3=in(7);
q4=in(8);
%Ojo con el punto elegido, porque si sin(q2) tenemos valores infinitos y
%por lo tanto nos encontramos con una particularidad.
if q2==0
    q2=pi/2;
end
p11=(2*cos(q1 + q2))/sin(q2);
p12=(2*sin(q1 + q2))/sin(q2);
p13=0; p14=0;

p21=-(2*(cos(q1 + q2) + cos(q1)))/sin(q2);
p22=-(2*(sin(q1 + q2) + sin(q1)))/sin(q2);
p23=0; p24=0;

p31=0; p32=0; p33=-1; p34=0;

p41=-(2*cos(q1))/sin(q2);
p42= -(2*sin(q1))/sin(q2);
p43=0; p44=1;

Ja1=[p11 p12 p13 p14];
Ja2=[p21 p22 p23 p24];
Ja3=[p31 p32 p33 p34];
Ja4=[p41 p42 p43 p44];
J_a=[Ja1; Ja2; Ja3; Ja4];

qd_1=J_a(1,1)*x1+J_a(1,2)*x2+J_a(1,3)*x3+J_a(1,4)*x4;
qd_2=J_a(2,1)*x1+J_a(2,2)*x2+J_a(2,3)*x3+J_a(2,4)*x4;
qd_3=J_a(3,1)*x1+J_a(3,2)*x2+J_a(3,3)*x3+J_a(3,4)*x4;
qd_4=J_a(4,1)*x1+J_a(4,2)*x2+J_a(4,3)*x3+J_a(4,4)*x4;


qd=[qd_1; qd_2; qd_3; qd_4];
