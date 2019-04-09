function [qdd] = modelo_dinam(in)
u=in(1:4);
q=in(5:8);
qd=in(9:12);
qdd_prev=in(13:16);

q1=q(1);q2=q(2);q3=q(3);q4=q(4);
q1d=qd(1);q2d=qd(2);q3d=qd(3);q4d=qd(4);

n =[conj(q1d)/10000 + conj(q1d)*((25*q1d*sin(2*q1))/64 - (51*q2d*sin(q2))/32) + conj(q2d)*((77*q2d*cos(2*q1 + 2*q2))/64 - (51*q1d*sin(q2))/32 + (51*2^(1/2)*q2d*cos(pi/4 + q2))/64 + (51*2^(1/2)*q2d*sin(pi/4 + 2*q1 + q2))/64);
    conj(q2d)/10000 + conj(q2d)*((77*q1d*sin(2*q1 + 2*q2))/64 + (77*q2d*sin(2*q1 + 2*q2))/64) + conj(q1d)*((77*q2d*sin(2*q1 + 2*q2))/64 + (51*q1d*sin(q2))/32 + (77*2^(1/2)*q1d*sin(pi/4 + 2*q1 + 2*q2))/64 + (51*2^(1/2)*q1d*sin(pi/4 + 2*q1 + q2))/32);
    25*conj(q3d) - 12753/100;
    2*conj(q4d)];
B=[(51*cos(q2))/16 - (25*cos(q1)^2)/32 + 128767/8000, (51*2^(1/2)*sin(pi/4 + q2))/64 - (51*2^(1/2)*cos(pi/4 + 2*q1 + q2))/64 - (77*2^(1/2)*cos(pi/4 + 2*q1 + 2*q2))/128 + 98019/16000,   1/4, 1707/1600;
   (51*2^(1/2)*sin(pi/4 + q2))/64 - (51*2^(1/2)*cos(pi/4 + 2*q1 + q2))/64 - (77*2^(1/2)*cos(pi/4 + 2*q1 + 2*q2))/128 + 98019/16000, (77*sin(q1 + q2)^2)/32 + 44197/8000,  53/4, 1707/1600;
    1/4,1/4, -25/2, 0;
    1707/1600,1707/1600, 0,-463/320];  

if (det(B)==0)
    qdd=qdd_prev;
else
    qdd=inv(B)*(u-n);
end

end