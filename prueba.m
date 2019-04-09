%%
%Punto 1.

%Parámetros
a1=0.5; a2=a1;
l1=0.25; l2=l1;

%D-H
A(1)=Link([0 0 a1 0 0]);
A(2)=Link([0 0 a2 pi 0]);
r=SerialLink(A);

q=[pi/2 -pi/2];
%while(q(1)>0)
%    r.plot(q);
%    q(1)=q(1)-pi/32;
%    q(2)=q(2)+pi/32;
%end

T=r.fkine(q); %crea la matriz de transformación homogénea
[V, D]=eig(Jacob*Jacob');

while(q(1)>0)
    axis([-6 6 -6 6 -6 6])
    elip=ellipsoid(a1*cos(q(1))+a2*cos(q(1)+q(2)), a1*sin(q(1))+a2*sin(q(1)+q(2)),0,M(4),M(3),0, 20)
    %r.plot(q);
    %plot(cos(q(1)+q(2)), r.maniplty(q,'yoshikawa'),'*')
    q(1)=q(1)-pi/32;
    q(2)=q(2)+pi/32;
    T=r.fkine(q); %crea la matriz de transformación homogénea
    M=svd(T); 

end 
%%
q=[pi/2 -pi/2];
i=1;
while q(1)>0
    R(i)=r.maniplty(q,'yoshikawa');
    q(1)=q(1)-pi/32;
    q(2)=q(2)+pi/32;
    i=i+1;
end
    

