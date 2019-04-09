%Transformacion de Jacobiana simbólica a numerica
function [xe]=JacobianaAnalitica (in)
qd1=in(1);
qd2=in(2);
qd3=in(3);
qd4=in(4);
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

p41=0;p42= 0;p43=0; p44=0;

Ja1=[p11 p12 p13 p14];
Ja2=[p21 p22 p23 p24];
Ja3=[p31 p32 p33 p34];
Ja4=[p41 p42 p43 p44];
J_a=[Ja1; Ja2; Ja3; Ja4];

xe_1=J_a(1,1)*qd1+J_a(1,2)*qd2+J_a(1,3)*qd3+J_a(1,4)*qd4;
xe_2=J_a(2,1)*qd1+J_a(2,2)*qd2+J_a(2,3)*qd3+J_a(2,4)*qd4;
xe_3=J_a(3,1)*qd1+J_a(3,2)*qd2+J_a(3,3)*qd3+J_a(3,4)*qd4;
xe_4=J_a(4,1)*qd1+J_a(4,2)*qd2+J_a(4,3)*qd3+J_a(4,4)*qd4;

xe=[xe_1; xe_2; xe_3; xe_4];