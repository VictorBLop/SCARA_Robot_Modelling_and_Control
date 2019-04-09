function w = fun_w(in)
q=in(1:4);
qd=in(5:8);
err_=in(9:12);
errd=in(13:16);
kp=in(17);
kd=in(18);
BM=in(19);
Q_=in(20:83);
QM=in(84);
alpha=in(85);

q1=q(1);q2=q(2);q3=q(3);q4=q(4);
q1d=qd(1);q2d=qd(2);q3d=qd(3);q4d=qd(4);

n =[conj(q1d)/10000 + conj(q1d)*((25*q1d*sin(2*q1))/64 - (51*q2d*sin(q2))/32) + conj(q2d)*((77*q2d*cos(2*q1 + 2*q2))/64 - (51*q1d*sin(q2))/32 + (51*2^(1/2)*q2d*cos(pi/4 + q2))/64 + (51*2^(1/2)*q2d*sin(pi/4 + 2*q1 + q2))/64);
    conj(q2d)/10000 + conj(q2d)*((77*q1d*sin(2*q1 + 2*q2))/64 + (77*q2d*sin(2*q1 + 2*q2))/64) + conj(q1d)*((77*q2d*sin(2*q1 + 2*q2))/64 + (51*q1d*sin(q2))/32 + (77*2^(1/2)*q1d*sin(pi/4 + 2*q1 + 2*q2))/64 + (51*2^(1/2)*q1d*sin(pi/4 + 2*q1 + q2))/32);
    25*conj(q3d) - 12753/100;
    2*conj(q4d)];

Q=[Q_(1:8) Q_(9:16) Q_(17:24) Q_(25:32) Q_(33:40) Q_(41:48) Q_(49:56) Q_(57:64)];
err=[err_;errd];

K=[kp*eye(4) kd*eye(4)];
D=[zeros(4);eye(4)];
z=D'*Q*err;
O=norm(n);
p=(alpha*QM+alpha*norm(K)*norm(err)+BM*O)/(1-alpha);

if(norm(z)<0.1)
    w=[0; 0; 0; 0];
else
    w=p*z/norm(z);
end

end
