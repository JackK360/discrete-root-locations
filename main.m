%% drawing boundaries for discrete root locations
clear all ; close all
%% performance constrains
ts0 = 3; %maximum settling time [s]
Mp0=.20;
tr0=1;
%% hardware constraints
f_s=5; %sampling frequency [hz]
omega_s=2*pi*f_s; %sampling frequency 
T=1/f_s ;% sampling period [s]
theta=(0:pi/50:2*pi)';
real_unit = cos(theta);
imag_unit=sin(theta);

figure; hold on;
p1 = plot(real_unit,imag_unit,'r--','linewi',2);
axis([-1.2 1.2 -1.2 1.2]), axis equal
title('discrete root locations','FontSize',16)
xlabel('real axis','FontSize',16)
ylabel('imaginary axis','FontSize',16)
ax=gca;
ax.FontSize = 16;

%% settling time constraint
mag_ts=exp(-(4/ts0)*T);

real_ts=mag_ts*cos(theta);
imag_ts=mag_ts*sin(theta);

p2=fill(real_ts,imag_ts,'g','LineStyle','none');
alpha(0.25);

%% overshoot
theta2=(0:pi/50:pi)';
zeta_Mp=sqrt(log(Mp0)^2/(pi^2+log(Mp0)^2));
mag_Mp = exp(-zeta_Mp/sqrt(1-zeta_Mp^2)*theta2);

real_Mp=mag_Mp.*cos(theta2);
imag_Mp= mag_Mp.*sin(theta2);

real_Mp_2=[real_Mp; flipud(real_Mp)];
imag_Mp_2 = [imag_Mp; flipud(-imag_Mp)];


p3=fill(real_Mp_2,imag_Mp_2,'b','LineStyle','none');
alpha(0.25)

%% rise time
zeta_tr = linspace(0,0.99,501)';
beta= atan(sqrt(1-zeta_tr.^2)./zeta_tr);
omegan_tr=(pi-beta)./(tr0*sqrt(1-zeta_tr.^2));
omegad_tr = omegan_tr.*sqrt(1-zeta_tr.^2);

mag_tr = exp(-zeta_tr.*omegan_tr*T);
ang_tr=omegad_tr*T;

real_tr=mag_tr.*cos(ang_tr);
imag_tr=mag_tr.*sin(ang_tr);

real_tr_2=[flipud(real_tr);1*cos((ang_tr(1):0.1:pi)')]
imag_tr_2=[flipud(imag_tr);1*sin((ang_tr(1):0.1:pi)')]

real_tr_3=[real_tr_2;flipud(real_tr_2)];
imag_tr_3=[imag_tr_2;flipud(-imag_tr_2)];

p4=fill(real_tr_3,imag_tr_3,'r-','LineStyle','none');
alpha(.25)

legend([p1,p2,p3,p4],{'stability condition',...
                        't_s condition',...
                        'M_p Condition',...
                        't_r condition'})
                    
legend('boxoff')
zgrid
