%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% Generador de trayectorias %%%%%%%%
%%%%%%%%%   Trayectoria circular     %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%Matrices de rotación
syms om1 om2 om3
Rx=[1 0 0; 0 cos(om1) -sin(om1); 0 sin(om1) cos(om1)];
Ry=[cos(om2) 0 sin(om2); 0 1 0; -sin(om2) 0 cos(om2)];
Rz=[cos(om3) -sin(om3) 0; sin(om3) cos(om3) 0; 0 0 1];

giro=Rz*Ry*Rx;

m1=(r(2,3)-r(1,3))/(r(2,2)-r(1,2));
om1=acos(ur(1)); %Giro x
m2=(r(2,3)-r(1,3))/(r(2,1)-r(1,1));
om2=acos(ur(2)); %Giro y
m3=(r(2,2)-r(1,2))/(r(2,1)-r(1,1));
om3=acos(ur(3)); %Giro z

%%
%Parámetros recibidos por el cliente:


%Vector unidad r del eje del círculo
r=[[0.25 0.5]' [0.25 0.6]' [0.25 1]']; 
ur=[r(2,1)-r(1,1) r(2,2)-r(1,2) r(2,3)-r(1,3)];
modulo=sqrt(ur(1)^2 + ur(2)^2 + ur(3)^2);
ur=ur/modulo;

%Vector posición pj de un punto en el círculo
pj=[0.3 -0.1 0.2];

%Recta en la dirección del eje (con origen (0,0,0))
i=-1;
w=1;
while(i<=2)
    eje(w,:)=[i*ur(1),i*ur(2),i*ur(3)];
    i=i+0.25;
    w=w+1;
end

%Vector posición d de un punto a lo largo del eje
d=[eje(9,1)+r(1,1), eje(9,2)+r(1,2), eje(9,3)+r(1,3)];%Le sumamos la posición inicial del vector r.,
%para que pertenezca 

%Cálculos para hallar el centro
syms p p_prima s
sgm = pj -d; %
%mod_sgm=sqrt(sgm(1)^2 +sgm(2)^2+sgm(3)^2);
a=(abs(sgm*ur')<sqrt(sgm(1)^2+sgm(2)^2+sgm(3)^2))
c = d + ((sgm*ur')*ur); %Centro del circulo (respecto 0xyz)

%Plano del círculo
normal=[ur(1) ur(2) ur(3)]; %vector normal del plano
x_p=pj(1); y_p=pj(2); z_p=pj(3);
A=normal(1); B=normal(2); C=normal(3);
D= -A*x_p -B*y_p -C*z_p;
%Fijamos zc;
zc=c(3); m=-D-C*zc;
%Para xc, y x(pj).
x_p=[c(1) pj(1)];
for i=1:2
    y_p(i)=m/B-(A/B)*x_p(i);
end
%Vector y
vec_y=[x_p(2)-x_p(1) y_p(2)-y_p(1) 0];
mod_vec=sqrt(vec_y(1)^2+vec_y(2)^2);
vec_y=vec_y/mod_vec;

vec_x=cross(vec_y,normal);
R=[vec_x' vec_y' normal'];

%Puntos respecto al eje x'y'z' (p_prima) y al eje xyz (pos)

p_prima=[];
p=sqrt((c(1)-pj(1))^2+(c(2)-pj(2))^2+(c(1)-pj(1))^2 );
N=60;
s=0;
for i=1:N
    p_prima(i,:)=[p*cos(s/p) p*sin(s/p) 0];
    G=(R*p_prima')';
    pos(i,:)=[c(1)+G(i,1) c(2)+G(i,2) c(3)+G(i,3)];
    s=s+p/4;
end

%Giro del círculo / Gráfica para observar la posición de los elementos
figure(1)
plot3(eje(:,1)+r(1,1),eje(:,2)+r(1,2),eje(:,3)+r(1,3),'-*','Color','green')
grid on
hold on
plot3(pj(1),pj(2),pj(3),'*','Color','black');
plot3(d(1),d(2),d(3),'*','Color','blue');
plot3(c(1),c(2),c(3),'*','Color','magenta');
plot3([r(1,1) ur(1)+r(1,1)],[r(1,2) ur(2)+r(1,2)],[r(1,3) ur(3)+r(1,3)],'-*','Color','cyan');


%Robot
%Parámetros
a1=0.5; a2=a1;
l1=0.25;l2=l1;
d0=1;
%Establecemos posiciones x,y,z
for n=1:N
    posx(n)=pos(n,1);
    posy(n)=pos(n,2);
    posz(n)=pos(n,3);
end

for n=1:N    
     cos2=(posx(n)^2+posy(n)^2-a1^2-a2^2)/(2*a1*a2);
     sin2=sqrt(1-cos2^2);
     Q2(n)=(atan2(sin2,cos2));  %Q2 mediante inversión cinemática
     
     sin1=(((a1+a2*cos2)*posy(n)-a2*sin2*posx(n))/(posx(n)^2+posy(n)^2));
     cos1=(((a1+a2*cos2)*posx(n)+a2*sin2*posy(n))/(posx(n)^2+posy(n)^2));
     Q1(n)=(atan2(sin1,cos1));  %Q1 mediante inversión cinemática
     
     Q3(n)=-l1-l2-posz(n);   %Q3 mediante inversión cinemática
    
end 

for n=1:N
    hold on
    figure(1)
    plot3([0 0], [0 0], [0 1])
    plot3([0 a1*cos(Q1(n))], [0 a1*sin(Q1(n))], [1 1]);
    plot3([a1*cos(Q1(n)) a1*cos(Q1(n))+a2*cos(Q1(n)+Q2(n))], [a1*sin(Q1(n)) a1*sin(Q1(n))+a2*sin(Q1(n)+Q2(n))], [1 1]);
    plot3([a1*cos(Q1(n))+a2*cos(Q1(n)+Q2(n)) a1*cos(Q1(n))+a2*cos(Q1(n)+Q2(n))], [a1*sin(Q1(n))+a2*sin(Q1(n)+Q2(n)) a1*sin(Q1(n))+a2*sin(Q1(n)+Q2(n))], [1 -0.5-Q3(n)]);
    plot3(pos(:,1),pos(:,2), pos(:,3),'-*')
    grid on
end
plot3(pos(:,1), pos(:,2),pos(:,3),'*','Color','r');xlabel('Eje X');ylabel('Eje Y');zlabel('Eje Z');

k=20; %número de puntos de la elipsoide
[x,y,z]=ellipsoid(0,0,0,0.1,0.1,0,k);
A=[];

for i=1:k+1
    A=[A;(R*[x(:,i)';y(:,i)';z(:,i)'])'];
end 
figure(2)
hold on
plot3(A(:,1)+r(1,1), A(:,2)+r(1,2),A(:,3)+r(1,3),'r');xlabel('Eje X');ylabel('Eje Y');zlabel('Eje Z');

hold on
plot3(r(:,1), r(:,2), r(:,3),'-*')
