%Interpolating polynomials with computed velocities at path points
%Generador de Trayectorias
function [x, y, z, fi]=GenTray(in)

pix=in(1);     %Posicion x inicial
piy=in(2);      %Posicion y inicial
piz=in(3);      %Posicion z inicial
phii=in(4)      %Orientación Phi inicial

pfx=in(5);      %Posicion x final
pfy=in(6);      %Posicion y final
pfz=in(7);      %Posicion z final
phif=in(8);     %Orientación Phi final

N=in(9);        %Numero de puntos de la trayectoria
tini=in(10);     %Tiempo Inicial
duracion=in(11);        %Duracion de la trayectoria
t=in(12);        %reloj

%Parámetros
a1=0.5; a2=a1;
l1=0.25;l2=l1;
d0=1;

T=duracion/(N-1); %intervalo de tiempo que transcurre en llegar de un punto intermedio a otro

%Inicializo los vectores de trayectorias, con el punto inicial como primer
%elemento y el final como último
posx=zeros(1,N);posx(1)=pix;posx(N)=pfx;
posy=zeros(1,N);posy(1)=piy;posy(N)=pfy;
posz=zeros(1,N);posz(1)=piz;posz(N)=pfz;
posphi=zeros(1,N);posphi(1)=phii;posphi(N)=phif;

%Calculo el incremento en cada eje
IncX=abs(pfx-pix);
IncY=abs(pfy-piy);
IncZ=abs(pfz-piz);
IncPhi=abs(phif-phii)

%Calculo los puntos intermedios por los que tiene que pasar
for n=2:N-1
    posx(n)=pix+sign(pfx-pix)*(n-1)*(IncX/(N-1));
    posy(n)=piy+sign(pfy-piy)*(n-1)*(IncY/(N-1));
    posz(n)=piz+sign(pfz-piz)*(n-1)*(IncZ/(N-1));
    posphi(n)=phii+sign(phif-phii)*(n-1)*(IncPhi/(N-1));
end
    pos=[posx' posy' posz' posphi'];

%Conocidos los puntos en cartesianas, los pasamos a articulares
for n=1:N    
     cos2=(posx(n)^2+posy(n)^2-a1^2-a2^2)/(2*a1*a2);
     sin2=sqrt(1-cos2^2);
     Q2(n)=(atan2(sin2,cos2));  %Q2 mediante inversión cinemática
     
     sin1=(((a1+a2*cos2)*posy(n)-a2*sin2*posx(n))/(posx(n)^2+posy(n)^2));
     cos1=(((a1+a2*cos2)*posx(n)+a2*sin2*posy(n))/(posx(n)^2+posy(n)^2));
     Q1(n)=(atan2(sin1,cos1));  %Q1 mediante inversión cinemática
     
     Q3(n)=-l1-l2-posz(n);   %Q3 mediante inversión cinemática
     Q4(n)=0;
     posq(n,:)=[Q1(n); Q2(n); Q3(n); Q4(n)];
end 

%Condiciones de contorno. Velocidades y aceleraciones inicial y final
vq(1,1) =0;     aq(1,1) =0;
vq(1,2) =0;     aq(1,2) =0;
vq(1,3) =0;     aq(1,3) =0;
vq(N,1) =0;     aq(N,1) =0;
vq(N,2) =0;     aq(N,2) =0;
vq(N,3) =0;     aq(N,3) =0;

%Velocidades intermedias
for j=1:3 %q1, q2, q3 e q4..... columnas
    for n=2:N-1 %filas......numero de puntos de la trayectoria
        if(sign((posq(n,j))-(posq(n-1,j)))==sign((posq(n+1,j))-(posq(n,j))))
            vq(n,j)=((posq(n+1,j))-(posq(n-1,j)))/(2*T);
        else
            vq(n,j)=0;
        end
    end
end

%Cinemática Directa para comprobación
for n=1:N
    Px(n)=a2*cos(Q1(n) + Q2(n)) + a1*cos(Q1(n));
    Py(n)=a2*sin(Q1(n) + Q2(n)) + a1*sin(Q1(n));
    Pz(n)=- l1 - l2 - Q3(n);
    cin_dir(n,:)=[Px(n);Py(n);Pz(n)];
end

q4=zeros(1,N);
%Q1_s=Q1;
%Q2_s=Q2;
%Q3_s=Q3;

%%
%Robot 3D
%Q1=eval(Q1_s);
%Q2=eval(Q2_s);
%Q3=eval(Q3_s);

%Robot en 3D y trayectoria
plot3(pix,piy,piz,'*black');legend('Posizione iniziale');
title('Traiettoria lineare (0.4,0,0.2) a (0.6,0.1,0.5)');
%axis([0.4 0.7 0 0.2 0.1 1]); 
hold on; grid on;
plot3(pfx,pfy,pfz,'*cyan');
xlabel('Asse X m)'); ylabel('Asse Y (m)'); zlabel('Asse Z (m)');
for n=1:N
    hold on
    figure(1)
    plot3([0 0], [0 0], [0 1])
    plot3([0 a1*cos(Q1(n))], [0 a1*sin(Q1(n))], [1 1]);
    plot3([a1*cos(Q1(n)) a1*cos(Q1(n))+a2*cos(Q1(n)+Q2(n))], [a1*sin(Q1(n)) a1*sin(Q1(n))+a2*sin(Q1(n)+Q2(n))], [1 1]);
    plot3([a1*cos(Q1(n))+a2*cos(Q1(n)+Q2(n)) a1*cos(Q1(n))+a2*cos(Q1(n)+Q2(n))], [a1*sin(Q1(n))+a2*sin(Q1(n)+Q2(n)) a1*sin(Q1(n))+a2*sin(Q1(n)+Q2(n))], [1 -0.5-Q3(n)]);
    plot3(pos(:,1),pos(:,2), pos(:,3),'-*')
end
plot3(pix,piy,piz,'*black');legend('Posizione iniziale');
plot3(pfx,pfy,pfz,'*cyan');
legend('Posizione iniziale','Posizione finale');
    

%Posición X,Y y Z respecto al tiempo
figure(5)
subplot(3,1,1);plot(t, pos(:,1),'-*','Color','b');grid on;xlabel('tempo (s)');ylabel('Pos. X (m)');title('Posizione X, Y e Z');
subplot(3,1,2);plot(t, pos(:,2),'-*','Color','r');grid on;xlabel('tempo (s)');ylabel('Pos. Y (m)')
subplot(3,1,3);plot(t, pos(:,3),'-*','Color','m');grid on;xlabel('tempo (s)');ylabel('Pos. Z (m)')

%Valores de las articulaciones y sus derivadas (velocidades)
%Articulación q1
figure(2)
subplot(3,1,1);plot(t, posq(:,1),'-*','Color','r');grid on;xlabel('tempo (s)'); ylabel('Q_{1} (rad)');title('Articolazione 1');
subplot(3,1,2);plot(t, vq(:,1),'-*','Color','r');grid on;xlabel('tempo (s)'); ylabel('Q_{1}^{.} (rad/s)')
subplot(3,1,3);plot(t, aq(:,1),'-*','Color','r');ylabel('Q_{1}^{..} (rad/s^2)'); grid on;
%Articulación q2
figure(3)
subplot(3,1,1);plot(t, posq(:,2),'-*','Color','b');grid on;xlabel('tempo (s)'); ylabel('Q2 (rad)');title('Articolazione 2');
subplot(3,1,2);plot(t, vq(:,2),'-*','Color','b');grid on;xlabel('tempo (s)'); ylabel('Q2^{.} (rad/s)')
subplot(3,1,3);plot(t, aq(:,2),'-*','Color','b');ylabel('Q_{2}^{..} (rad/s^2)');grid on;
%Articulación q3
figure(4)
subplot(3,1,1);plot(t, posq(:,3),'-*','Color','m');grid on;xlabel('tempo (s)'); ylabel('Q_3 (rad)');title('Articolazione 3');
subplot(3,1,2);plot(t, vq(:,3),'-*','Color','m');grid on; xlabel('tempo (s)');ylabel('Q_{3}^{.} (rad/s)')
subplot(3,1,3);plot(t, aq(:,3),'-*','Color','m');ylabel('Q_{3}^{..} (rad/s^2)');grid on;
%Articulación q4
figure(5)
subplot(3,1,1);plot(t, q4,'-*','Color','black');grid on;xlabel('tempo (s)'); ylabel('Q_4 (rad)');title('Articolazione 4');
subplot(3,1,2);plot(t, q4,'-*','Color','black');grid on; xlabel('tempo (s)');ylabel('Q_{4}^{.} (rad/s)')
subplot(3,1,3);plot(t, q4,'-*','Color','black');ylabel('Q_{4}^{..} (rad/s^2)');grid on;
%%
%%%%%%%%%%%%%%%%%%%%%
%%%%   TIEMPO   %%%%%
%%%%%%%%%%%%%%%%%%%%%

%Si aun no estamos dentro del tiempo en el que tengo que calcular la
%trayectoria, tendremos las condiciones iniciales
if(t<=tini)
     trayectoria = [posq(1,1);posq(1,2);posq(1,3);vq(1,1);vq(1,2);vq(1,3);aq(1,1);aq(1,2);aq(1,3)];
    return;
end

%Si ya hemos terminado la trayectoria
if(t>=tini+duracion)
     trayectoria = [posq(N,1);posq(N,2);posq(N,3);vq(N,1);vq(N,2);vq(N,3);aq(N,1);aq(N,2);aq(N,3)];    
     return;
end 


%Si se ha llegado hasta aquí, estamos dentro de la trayectoria
%Identificamos el tramo:
Tramo=1;
while((t-tini)>=(T*Tramo))
    Tramo=Tramo+1;
end


%Antes de llamar a la funcion, tenemos que conocer el tiempo que lleva en
%el tramo
time=t-tini-T*(Tramo-1);        %tiempo que lleva en el tramo

for j=1:3
q(:,j)=Coef([posq(Tramo,j); posq(Tramo+1,j); vq(Tramo,j); vq(Tramo+1,j); T; time]);
end
    
%Se devuelve q1,q2,q3,vq1,vq2,vq3,aq1,aq2,aq3:
trayectoria=[q(1,1); q(1,2); q(1,3); q(2,1); q(2,2); q(2,3); q(3,1); q(3,2); q(3,3)];
return;