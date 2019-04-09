function q = CinematicaInversa(in)
x=in(1);
y=in(2);
z=in(3);
phi=in(4);

a1=0.5; a2=a1;
l1=0.25; l2=l1;

cos2=(x^2+y^2-a1^2-a2^2)/(2*a1*a2);
sin2=sqrt(1-cos2^2);
q2=atan2(sin2,cos2);

sin1=(((a1+a2*cos2)*y-a2*sin2*x)/(x^2+y^2));
cos1=(((a1+a2*cos2)*x+a2*sin2*y)/(x^2+y^2));
q1=atan2(sin1,cos1);

q3=-l1-l2-z;

q4=phi+q1+q2;

q=[q1 q2 q3 q4];


