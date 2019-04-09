
function [xyphi]=CinematicaDirectaPseudo(in)

q1=in(1);
q2=in(2);
q3=in(3);
q4=in(4);

%Parámetros
a1=0.5; a2=a1;
l1=0.25; l2=l1;

px=a2*cos(q1 + q2) + a1*cos(q1);
py=a2*sin(q1 + q2) + a1*sin(q1);
%pz=- l1 - l2 - q3;
phi=-q1-q2+q4;
xyphi=[px;py;phi];
