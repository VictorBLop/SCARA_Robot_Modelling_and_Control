function [q]=Euler(in)
%qd è una matrice con le derivate de q1,q2,q3 e q4.
%q1 è la posizione articolare anteriore (q(t)).
%q11 è la posizione articolare prossima (q(t+1)).
qd1=in(1); 
qd2=in(2);
qd3=in(3);
qd4=in(4);
q1=in(5); 
q2=in(6); 
q3=in(7); 
q4=in(8); 
dif_t=in(9);%constant

q11=q1+qd1*dif_t;
q22=q2+qd2*dif_t;
q33=q3+qd3*dif_t;
q44=q4+qd4*dif_t;
q=[q11;q22;q33;q44];
