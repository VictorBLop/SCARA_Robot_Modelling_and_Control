function variables = Coef(in)

% Variables de entrada en la funcion: [qi qf vi vf T t]
qi       = in(1);      %posicion inicial del tramo
qf       = in(2);      %posicion final del tramo
vi       = in(3);      %velocidad inicial del tramo
vf       = in(4);      %velocidad final del tramo
T        = in(5);      %Duracion del tramo
t        = in(6);      %Tiempo actual

%Calculamos los coeficientes del tramo
d=qi;
c=vi;
a=(c*T+2*d+T*vf-2*qf)/(T^3);
b=(vf-3*a*T*T -c)/(2*T);

%calculamos la posicion, velocidad y aceleracion
qp= a*t^3 + b*t^2 + c*t + d;
%qv= 3*a*t^2 + 2*b*t + c;
%qa= 6*a*t + 2*b;

variables=qp;