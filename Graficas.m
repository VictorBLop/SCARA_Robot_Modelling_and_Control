%% Gráficas %%
%% Punto 1. Manipulability Ellipsoids
%En rob_sic.

%% Punto 2 
% Traiettoria lineare
%Prima si deve eggecutare il programm 'GenTray.m'.
%Robot 3D
%Q1=eval(Q1_s);
%Q2=eval(Q2_s);
%Q3=eval(Q3_s);

%Robot en 3D y trayectoria
plot3(pix,piy,piz,'*black');legend('Posizione iniziale');
title('Traiettoria lineare [0.2,0.3,0.5] a [0.6, 0.2, 0.3]');
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
subplot(3,1,1);plot(t, eval(posq(:,1)),'-*','Color','r');grid on;xlabel('tempo (s)'); ylabel('Q_{1} (rad)');title('Articolazione 1');
subplot(3,1,2);plot(t, vq(:,1),'-*','Color','r');grid on;xlabel('tempo (s)'); ylabel('Q_{1}^{.} (rad/s)')
subplot(3,1,3);plot(t, aq(:,1),'-*','Color','r');ylabel('Q_{1}^{..} (rad/s^2)'); grid on;
%Articulación q2
figure(3)
subplot(3,1,1);plot(t, eval(posq(:,2)),'-*','Color','b');grid on;xlabel('tempo (s)'); ylabel('Q2 (rad)');title('Articolazione 2');
subplot(3,1,2);plot(t, vq(:,2),'-*','Color','b');grid on;xlabel('tempo (s)'); ylabel('Q2^{.} (rad/s)')
subplot(3,1,3);plot(t, aq(:,2),'-*','Color','b');ylabel('Q_{2}^{..} (rad/s^2)');grid on;
%Articulación q3
figure(4)
subplot(3,1,1);plot(t, eval(posq(:,3)),'-*','Color','m');grid on;xlabel('tempo (s)'); ylabel('Q_3 (rad)');title('Articolazione 3');
subplot(3,1,2);plot(t, vq(:,3),'-*','Color','m');grid on; xlabel('tempo (s)');ylabel('Q_{3}^{.} (rad/s)')
subplot(3,1,3);plot(t, aq(:,3),'-*','Color','m');ylabel('Q_{3}^{..} (rad/s^2)');grid on;
%Articulación q4
figure(5)
subplot(3,1,1);plot(t, q4,'-*','Color','black');grid on;xlabel('tempo (s)'); ylabel('Q_4 (rad)');title('Articolazione 4');
subplot(3,1,2);plot(t, q4,'-*','Color','black');grid on; xlabel('tempo (s)');ylabel('Q_{4}^{.} (rad/s)')
subplot(3,1,3);plot(t, q4,'-*','Color','black');ylabel('Q_{4}^{..} (rad/s^2)');grid on;

% Punto 3

%% JACOBIANA INVERSA %%

%TRAIETTORIA es el valor xyzphi deseado ("xd").
%VELOCITA es el valor xyzphi_d deseado ("xd*"). (La derivada de
%traiettoria).
%XE es la posición xyzphi actual. (xe)
%q es el valor de las articulaciones (q).
%qd es el valor de la velocidad de las articulaciones (qd).

%Articulación q1
figure(2)
subplot(2,1,1);plot(t, q(:,1));grid on;xlabel('tempo (s)'); ylabel('Q_{1} (rad)');title('Articolazione 1');
legend('Articolazione Q_{1}');
subplot(2,1,2);plot(t, qd(:,1));grid on;xlabel('tempo (s)'); ylabel('Q_{1}^{.} (rad/s)');
title('Velocità 1');legend('Velocità Q_{1}^.');saveas(gcf,'ArtVel 1','emf');

%Articulación q2
figure(3)
subplot(2,1,1);plot(t, q(:,2));grid on;xlabel('tempo (s)'); ylabel('Q_2 (rad)');title('Articolazione 2');
legend('Articolazione Q_{2}');
subplot(2,1,2);plot(t, qd(:,2));grid on;xlabel('tempo (s)'); ylabel('Q_2^{.} (rad/s)');
title('Velocità 2');legend('Velocità Q_{2}^.');saveas(gcf,'ArtVel 2','emf');

%Articulación q3
figure(4)
subplot(2,1,1);plot(t, q(:,3));grid on;xlabel('tempo (s)'); ylabel('Q_3 (rad)');title('Articolazione 3');
legend('Articolazione Q_{3}');
subplot(2,1,2);plot(t, qd(:,3));grid on; xlabel('tempo (s)');ylabel('Q_{3}^{.} (rad/s)');
title('Velocità 3');legend('Velocità Q_{3}^.');saveas(gcf,'ArtVel 3','emf');

%Articulación q4
figure(5)
subplot(2,1,1);plot(t, q(:,4));grid on;xlabel('tempo (s)'); ylabel('Q_4 (rad)');title('Articolazione 4');
legend('Articolazione Q_{4}');
subplot(2,1,2);plot(t, qd(:,4));grid on; xlabel('tempo (s)');ylabel('Q_{4}^{.} (rad/s)');
title('Velocità 4');legend('Velocità Q_{4}^.');saveas(gcf,'ArtVel 4','emf');

%Posizione X, Y e Z
figure(6)
plot(t,xe(:,1),'r',t,traiettoria(:,1),'LineWidth',2);xlabel('tempo (s)'); ylabel('Pos.X (m)');title('Posizione X');grid on
legend('Posizione X','Riferimento X_{d}');saveas(gcf,'Posizione X','emf');

figure(7)
plot(t,xe(:,2),t,traiettoria(:,2),'LineWidth',2);xlabel('tempo (s)'); ylabel('Pos.Y (m)');title('Posizione Y');grid on
legend('Posizione Y','Riferimento Y_{d}');saveas(gcf,'Posizione Y','emf');

figure(8)
plot(t,xe(:,3),t,traiettoria(:,3),'LineWidth',2);xlabel('tempo (s)'); ylabel('Pos.Z (m)');title('Posizione Z');grid on
legend('Posizione Z','Riferimento Z_{d}');saveas(gcf,'Posizione Z','emf');

figure(9)
plot(t,xe(:,4),t,traiettoria(:,4),'LineWidth',2);xlabel('tempo (s)'); ylabel('Orientamento Phi(z) (rad)');title('Orientamento Phi(z)');grid on
legend('Orientamento Phi(z)','Riferimento Phi_{d}(z)');saveas(gcf,'Orientamento Z','emf');

%% JACOBIANA TRASPOSTA %%

%TRAIETTORIA es el valor xyzphi deseado ("xd").
%VELOCITA es el valor xyzphi_d deseado ("xd*"). (La derivada de
%traiettoria).
%XE es la posición xyzphi actual. (xe)
%q es el valor de las articulaciones (q).
%qd es el valor de la velocidad de las articulaciones (qd).

%Articulación q1
figure(2)
subplot(2,1,1);plot(t, q_tr(:,1));grid on;xlabel('tempo (s)'); ylabel('Q_{1} (rad)');title('Articolazione 1');
legend('Articolazione Q_{1}');
subplot(2,1,2);plot(t, qd_tr(:,1));grid on;xlabel('tempo (s)'); ylabel('Q_{1}^{.} (rad/s)');
title('Velocità 1');legend('Velocità Q_{1}^.');saveas(gcf,'ArtVel 1','emf');

%Articulación q2
figure(3)
subplot(2,1,1);plot(t, q_tr(:,2));grid on;xlabel('tempo (s)'); ylabel('Q_2 (rad)');title('Articolazione 2');
legend('Articolazione Q_{2}');
subplot(2,1,2);plot(t, qd_tr(:,2));grid on;xlabel('tempo (s)'); ylabel('Q_2^{.} (rad/s)');
title('Velocità 2');legend('Velocità Q_{2}^.');saveas(gcf,'ArtVel 2','emf');

%Articulación q3
figure(4)
subplot(2,1,1);plot(t, q_tr(:,3));grid on;xlabel('tempo (s)'); ylabel('Q_3 (rad)');title('Articolazione 3');
legend('Articolazione Q_{3}');
subplot(2,1,2);plot(t, qd_tr(:,3));grid on; xlabel('tempo (s)');ylabel('Q_{3}^{.} (rad/s)');
title('Velocità 3');legend('Velocità Q_{3}^.');saveas(gcf,'ArtVel 3','emf');

%Articulación q4
figure(5)
subplot(2,1,1);plot(t, q_tr(:,4));grid on;xlabel('tempo (s)'); ylabel('Q_4 (rad)');title('Articolazione 4');
legend('Articolazione Q_{4}');
subplot(2,1,2);plot(t, qd_tr(:,4));grid on; xlabel('tempo (s)');ylabel('Q_{4}^{.} (rad/s)');
title('Velocità 4');legend('Velocità Q_{4}^.');saveas(gcf,'ArtVel 4','emf');

%Posizione X, Y e Z
figure(6)
plot(t,xe_tr(:,1),'r',t,traiettoria_tr(:,1),'LineWidth',2);xlabel('tempo (s)'); ylabel('Pos.X (m)');title('Posizione X');grid on
legend('Posizione X','Riferimento X_{d}');saveas(gcf,'Posizione X','emf');

figure(7)
plot(t,xe_tr(:,2),t,traiettoria_tr(:,2),'LineWidth',2);xlabel('tempo (s)'); ylabel('Pos.Y (m)');title('Posizione Y');grid on
legend('Posizione Y','Riferimento Y_{d}');saveas(gcf,'Posizione Y','emf');

figure(8)
plot(t,xe_tr(:,3),t,traiettoria_tr(:,3),'LineWidth',2);xlabel('tempo (s)'); ylabel('Pos.Z (m)');title('Posizione Z');grid on
legend('Posizione Z','Riferimento Z_{d}');saveas(gcf,'Posizione Z','emf');

figure(9)
plot(t,xe_tr(:,4),t,traiettoria_tr(:,4),'LineWidth',2);xlabel('tempo (s)'); ylabel('Orientamento Phi(z) (rad)');title('Orientamento Phi(z)');grid on
legend('Orientamento Phi(z)','Riferimento Phi_{d}(z)');saveas(gcf,'Orientamento Z','emf');


%% JACOBIANA PSEUDOINVERSA %%
%Articulación q1
figure(2)
subplot(2,1,1);plot(t, q_p(:,1));grid on;xlabel('tempo (s)'); ylabel('Q_{1} (rad)');title('Articolazione 1');
legend('Articolazione Q_{1}');
subplot(2,1,2);plot(t, qd_p(:,1));grid on;xlabel('tempo (s)'); ylabel('Q_{1}^{.} (rad/s)');
title('Velocità 1');legend('Velocità Q_{1}^.');saveas(gcf,'ArtVel 1','emf');

%Articulación q2
figure(3)
subplot(2,1,1);plot(t, q_p(:,2));grid on;xlabel('tempo (s)'); ylabel('Q_2 (rad)');title('Articolazione 2');
legend('Articolazione Q_{2}');
subplot(2,1,2);plot(t, qd_p(:,2));grid on;xlabel('tempo (s)'); ylabel('Q_2^{.} (rad/s)');
title('Velocità 2');legend('Velocità Q_{2}^.');saveas(gcf,'ArtVel 2','emf');

%Articulación q3
figure(4)
subplot(2,1,1);plot(t, q_p(:,3));grid on;xlabel('tempo (s)'); ylabel('Q_3 (rad)');title('Articolazione 3');
legend('Articolazione Q_{3}');
subplot(2,1,2);plot(t, qd_p(:,3));grid on; xlabel('tempo (s)');ylabel('Q_{3}^{.} (rad/s)');
title('Velocità 3');legend('Velocità Q_{3}^.');saveas(gcf,'ArtVel 3','emf');

%Articulación q4
figure(5)
subplot(2,1,1);plot(t, q_p(:,4));grid on;xlabel('tempo (s)'); ylabel('Q_4 (rad)');title('Articolazione 4');
legend('Articolazione Q_{4}');
subplot(2,1,2);plot(t, qd_p(:,4));grid on; xlabel('tempo (s)');ylabel('Q_{4}^{.} (rad/s)');
title('Velocità 4');legend('Velocità Q_{4}^.');saveas(gcf,'ArtVel 4','emf');

%Posizione X, Y e Z
figure(6)
plot(t,xe_p(:,1),'r',t,traiettoria_p(:,1),'LineWidth',2);xlabel('tempo (s)'); ylabel('Pos.X (m)');title('Posizione X');grid on
legend('Posizione X','Riferimento X_{d}');saveas(gcf,'Posizione X','emf');

figure(7)
plot(t,xe_p(:,2),t,traiettoria_p(:,2),'LineWidth',2);xlabel('tempo (s)'); ylabel('Pos.Y (m)');title('Posizione Y');grid on
legend('Posizione Y','Riferimento Y_{d}');saveas(gcf,'Posizione Y','emf');

figure(8)
plot(t,xe_p(:,3),t,traiettoria_p(:,3),'LineWidth',2);xlabel('tempo (s)'); ylabel('Orientamento Phi(z) (rad)');title('Orientamento Phi(z)');grid on
legend('Orientamento Phi(z)','Riferimento Phi_{d}(z)');saveas(gcf,'Orientamento Z','emf');

%% CONTROLLO ROBUSTO %%
figure(1)
subplot(2,2,1);plot(t,q(1:length(t),1),'b',t,qr(1:length(t),1),'r','LineWidth',2);
xlabel('tempo(s)'); ylabel('Articolazione Q_{1}(rad)');title('Articolazione Q_1');grid on
legend('Articolazione Q_1','Riferimento Q_{1r}');
subplot(2,2,2);plot(t,q(1:length(t),2),'g',t,qr(1:length(t),2),'r','LineWidth',2);
xlabel('tempo(s)'); ylabel('Articolazione Q_{2}(rad)');title('Articolazione Q_2');grid on
legend('Articolazione Q_2','Riferimento Q_{2r}');
subplot(2,2,3);plot(t,q(1:length(t),3),'y',t,qr(1:length(t),3),'r','LineWidth',2);
xlabel('tempo(s)'); ylabel('Articolazione Q_{3}(rad)');title('Articolazione Q_3');grid on
legend('Articolazione Q_3','Riferimento Q_{3r}');
subplot(2,2,4);plot(t,q(1:length(t),4),'c',t,qr(1:length(t),4),'r','LineWidth',2);
xlabel('tempo(s)'); ylabel('Articolazione Q_{4}(rad)');title('Articolazione Q_4');grid on
legend('Articolazione Q_4','Riferimento Q_{4r}');saveas(gcf,'Articolazione Q','emf');

figure(2)
subplot(2,2,1);plot(t,qd(1:length(t),1),'b',t,qdr(1:length(t),1),'r','LineWidth',2);
xlabel('tempo(s)'); ylabel('Velocità Q_{1}^{.}(rad/s)');title('Velocità Q_{1}^{.}');grid on
legend('Velocità Q_1^{.}','Riferimento Q_{1r}^{.}');
subplot(2,2,2);plot(t,qd(1:length(t),2),'g',t,qdr(1:length(t),2),'r','LineWidth',2);
xlabel('tempo(s)'); ylabel('Velocità Q_{2}^{.}(rad/s)');title('Velocità Q_{2}^{.}');grid on
legend('Velocità Q_2^{.}','Riferimento Q_{2r}^{.}');
subplot(2,2,3);plot(t,qd(1:length(t),3),'y',t,qdr(1:length(t),3),'r','LineWidth',2);
xlabel('tempo(s)'); ylabel('Velocità Q_{3}^{.}(rad/s)');title('Velocità Q_{3}^{.}');grid on
legend('Velocità Q_3^{.}','Riferimento Q_{3r}^{.}');
subplot(2,2,4);plot(t,qd(1:length(t),4),'c',t,qdr(1:length(t),4),'r','LineWidth',2);
xlabel('tempo(s)'); ylabel('Velocità Q_{4}^{.}(rad/s)');title('Velocità Q_{4}^{.}');grid on
legend('Velocità Q_4^{.}','Riferimento Q_{4r}^{.}');saveas(gcf,'Velocità Q','emf');

figure(3)
subplot(2,2,1);plot(t,qd(1:length(t),1),'b',t,qdr(1:length(t),1),'r','LineWidth',2);
xlabel('tempo(s)'); ylabel('Accelerazione Q_{1}^{..}(rad/s^2)');title('Accelerazione Q_{1}^{..}');grid on
legend('Accelerazione Q_1^{..}','Riferimento Q_{1r}^{..}');
subplot(2,2,2);plot(t,qd(1:length(t),2),'g',t,qdr(1:length(t),2),'r','LineWidth',2);
xlabel('tempo(s)'); ylabel('Accelerazione Q_{2}^{..}(rad/s^2)');title('Accelerazione Q_{2}^{..}');grid on
legend('Accelerazione Q_2^{..}','Riferimento Q_{2r}^{..}');
subplot(2,2,3);plot(t,qd(1:length(t),3),'y',t,qdr(1:length(t),3),'r','LineWidth',2);
xlabel('tempo(s)'); ylabel('Accelerazione Q_{3}^{..}(rad/s^2)');title('Accelerazione Q_{3}^{..}');grid on
legend('Accelerazione Q_3^{..}','Riferimento Q_{3r}^{..}');
subplot(2,2,4);plot(t,qd(1:length(t),4),'c',t,qdr(1:length(t),4),'r','LineWidth',2);
xlabel('tempo(s)'); ylabel('Accelerazione Q_{4}^{..}(rad/s^2)');title('Accelerazione Q_{4}^{..}');grid on
legend('Accelerazione Q_4^{..}','Riferimento Q_{4r}^{..}');saveas(gcf,'Accelerazione Q','emf');
% subplot(2,1,2);plot(t,u(1:length(t),3),'m','LineWidth',2);
% xlabel('tempo(s)'); ylabel('Signale controllo u_{4}(rad)');grid on
% legend('Signale controllo u_4');





