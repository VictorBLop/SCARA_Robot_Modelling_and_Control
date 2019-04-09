%Transformacion de Jacobiana simbólica a numerica
x1=xi;
x2=yi;
x3=phii;
q1=0;
q2=pi/2;
q3=0;
q4=0;

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
% Ja3=[p31 p32 p33 p34];
Ja4=[p41 p42 p43 p44];
Ja=[Ja1; Ja2; Ja4];     %Obviando la componente z

%A partir de aqui he añadido yo
% syms q1_sym q2_sym q3_sym q4_sym
% q=[q1_sym q2_sym q3_sym q4_sym];
% q_=[0 0 0 0];
% q_M=[pi-pi/8 pi-pi/8 .5-.1 pi-pi/8];
% q_m=-[pi-pi/8 pi-pi/8 .5-.1 pi-pi/8];
% w=0;
% for k=[1:4]
%     w=w-(1/2*4)*((q(k)-q_(k))/(q_M(k)-q_m(k)))^2;
% end
k0=2;
% for k=[1:4]
%     q0_der(k,1)=k0*diff(w,q1_sym);
% end
% q0_der=eval(q0_der);
q0_der=-k0*[64/(49*pi^2)*q1, 64/(49*pi^2)*q2, 25/4*q3, 64/(49*pi^2)*q4]';
Ja*Ja'
Ja_inv=Ja'*(Ja*Ja')^-1;
ve=[x1 x2 x3]';

qd=double(Ja_inv*ve+(eye(4)-Ja_inv*Ja)*q0_der)
