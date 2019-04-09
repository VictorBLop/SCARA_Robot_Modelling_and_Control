%Transformacion de Jacobiana simbólica a numerica
function [qd]=JacobianaTrasposta (in)
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
p11=- sin(q1 + q2)/2 - sin(q1)/2;
p12=-sin(q1 + q2)/2;
p13=0; p14=0;

p21=cos(q1 + q2)/2 + cos(q1)/2;
p22=cos(q1 + q2)/2;
p23=0; p24=0;

p31=0; p32=0; p33=-1; p34=0;

p41=-1;
p42= -1;
p43=0; p44=1;

Ja1=[p11 p21 p31 p41];
Ja2=[p12 p22 p32 p42];
Ja3=[p13 p23 p33 p43];
Ja4=[p14 p24 p34 p44];
tras_J_a=[Ja1; Ja2; Ja3; Ja4];

qd_1=tras_J_a(1,1)*x1+tras_J_a(1,2)*x2+tras_J_a(1,3)*x3+tras_J_a(1,4)*x4;
qd_2=tras_J_a(2,1)*x1+tras_J_a(2,2)*x2+tras_J_a(2,3)*x3+tras_J_a(2,4)*x4;
qd_3=tras_J_a(3,1)*x1+tras_J_a(3,2)*x2+tras_J_a(3,3)*x3+tras_J_a(3,4)*x4;
qd_4=tras_J_a(4,1)*x1+tras_J_a(4,2)*x2+tras_J_a(4,3)*x3+tras_J_a(4,4)*x4;


qd=[qd_1 qd_2 qd_3 qd_4];
