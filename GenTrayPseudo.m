%Interpolating polynomials with computed velocities at path points
%Generador de Trayectorias
function [traiettoria]=GenTray(in)

pix=in(1);     %Posicion x inicial
piy=in(2);      %Posicion y inicial
phii=in(3);      %Orientazione Phi inicial
pfx=in(4);      %Posicion x final
pfy=in(5);      %Posicion y final
phif=in(6);     %Orientazione Phi final
N=in(7);        %Numero de puntos de la trayectoria
tini=in(8);    %Tiempo Inicial
duracion=in(9);%Duracion de la trayectoria
t=in(10);       %reloj

%Par�metros
a1=0.5; a2=a1;
l1=0.25;l2=l1;
d0=1;

T=duracion/(N-1); %intervalo de tiempo que transcurre en llegar de un punto intermedio a otro

%Inicializo los vectores de trayectorias, con el punto inicial como primer
%elemento y el final como �ltimo
posx=zeros(1,N);posx(1)=pix;posx(N)=pfx;
posy=zeros(1,N);posy(1)=piy;posy(N)=pfy;
%posz=zeros(1,N);posz(1)=piz;posz(N)=pfz;
posphi=zeros(1,N);posphi(1)=phii;posphi(N)=phif;

%Calculo el incremento en cada eje
IncX=abs(pfx-pix);
IncY=abs(pfy-piy);
%IncZ=abs(pfz-piz);
IncPhi=abs(phif-phii);

%Calculo los puntos intermedios por los que tiene que pasar
for n=2:N-1
    posx(n)=pix+sign(pfx-pix)*(n-1)*(IncX/(N-1));
    posy(n)=piy+sign(pfy-piy)*(n-1)*(IncY/(N-1));
    %posz(n)=piz+sign(pfz-piz)*(n-1)*(IncZ/(N-1));
    posphi(n)=phii+sign(phif-phii)*(n-1)*(IncPhi/(N-1));
end
    pos=[posx' posy' posphi'];
    
%Condiciones de contorno. Velocidades y aceleraciones inicial y final
v(1,1) =0;       
v(1,2) =0;     
v(1,3) =0;
v(N,1) =0;     
v(N,2) =0;     
v(N,3) =0; 


%Velocidades intermedias
for j=1:3 %q1, q2 y q4..... columnas
    for n=2:N-1 %filas......numero de puntos de la trayectoria
        if(sign(pos(n,j)-pos(n-1,j))==sign(pos(n+1,j)-pos(n,j)))
            v(n,j)=(pos(n+1,j)-pos(n-1,j))/(2*T);
        else
            v(n,j)=0;
        end
    end
end
%Si aun no estamos dentro del tiempo en el que tengo que calcular la
%trayectoria, tendremos las condiciones iniciales
if(t<=tini)
    traiettoria = [pos(1,1); pos(1,2); pos(1,3);0; 0; 0];
%Si ya hemos terminado la trayectoria
else if(t>=tini+duracion)
     traiettoria = [pos(N,1); pos(N,2); pos(N,3); 0; 0; 0];   
else
    %Si se ha llegado hasta aqu�, estamos dentro de la trayectoria
    %Identificamos el tramo:
    Tramo=1;
    while((t-tini)>=(T*Tramo))
        Tramo=Tramo+1;
    end


    %Antes de llamar a la funcion, tenemos que conocer el tiempo que lleva en
    %el tramo
    time=t-tini-T*(Tramo-1);        %tiempo que lleva en el tramo

    for j=1:3
        %Calculamos los coeficientes del tramo
        if (Tramo < N)
             vf=v(Tramo+1,j);
             qf=pos(Tramo+1,j);
        else
            vf=0;
            qf=0;
        end
        qi=pos(Tramo,j);
        vi=v(Tramo,j);
        d=qi;
        c=vi;
        a=(c*T+2*d+T*vf-2*qf)/(T^3);
        b=(vf-3*a*T*T -c)/(2*T);
    %calculamos la posicion, velocidad y aceleracion
        p(j)= a*time^3 + b*time^2 + c*time + d;    
        v(j)=3*a*time^2 +2*b*time + c;
    end

    traiettoria=[p(1); p(2); p(3); v(1); v(2); v(3)];
    end
end
