%Transformacion de Jacobiana simbólica a numerica
function [e]=JacobianaDerivada (in)
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
q1_sym=q1; q2_sym=q2; q3_sym=q3; q4_sym=q4;
q1d_sym=qd1; q2d_sym=qd2; q3d_sym=qd3; q4d_sym=qd4;

p11= -cos(q1_sym+q2_sym)*q1d_sym/2 -cos(q1_sym)*q1d_sym/2;
p12= -cos(q1_sym)*q1d_sym/2;
p13=0; p14=0;

p21= -sin(q1_sym+q2_sym)*q1d_sym/2 -sin(q1_sym)*q1d_sym/2;
p22= -sin(q1_sym)*q1d_sym/2;
p23=0; p24=0;

p31=-cos(q1_sym+q2_sym)*q2d_sym/2;
p32=-cos(q1_sym+q2_sym)*q2d_sym/2;
p33=0; p34=0;

p41=-sin(q1_sym+q2_sym)*q2d_sym/2;
p42=-sin(q1_sym+q2_sym)*q2d_sym/2;
p43=0; p44=0;

Jad1=[p11 p12 p13 p14];
Jad2=[p21 p22 p23 p24];
Jad3=[p31 p32 p33 p34];
Jad4=[p41 p42 p43 p44];
J_ad=[Jad1; Jad2; Jad3; Jad4];

e_1=J_ad(1,1)*qd1+J_ad(1,2)*qd2+J_ad(1,3)*qd3+J_ad(1,4)*qd4;
e_2=J_ad(2,1)*qd1+J_ad(2,2)*qd2+J_ad(2,3)*qd3+J_ad(2,4)*qd4;
e_3=J_ad(3,1)*qd1+J_ad(3,2)*qd2+J_ad(3,3)*qd3+J_ad(3,4)*qd4;
e_4=J_ad(4,1)*qd1+J_ad(4,2)*qd2+J_ad(4,3)*qd3+J_ad(4,4)*qd4;

e=[e_1; e_2; e_3; e_4];