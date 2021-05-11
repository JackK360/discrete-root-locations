clear all; clc;
%state feedback controller
A=[0 1; -.998 1.998];
B=[0 1]';
C=[.1998 .2005];
D=0;

zstar=[.992+.00384i,.992-.00384i];
K=acker(A,B,zstar)
A_CL=A-B*K
eig(A_CL)

betastar=[.9584+.06651i,.9584-.06651i];
L=acker(A',C',betastar)'

fs = 50;
T=1/fs;

y0=1;
x0=[1;-1];
N=750;
t=(0:T:(N-1)*T)';

y=zeros(1,N);
yhat=zeros(1,N);
x=zeros(2,N);
xhat=zeros(2,N);
u=zeros(1,N);

y(1)=y0;
yhat(1)=y0;
x(:,1)=x0
xhat(:,1)=[-1 1]';
u(1)=-K*xhat(:,1);

for k = 2:N
    x(:,k)=A*x(:,k-1)+B*(u(k-1));
    xhat(:,k)=A*xhat(:,k-1)+B*u(k-1)-L*(yhat(k-1)-y(k-1));
    u(k)=-K*xhat(:,k);
    yhat(k)=C*xhat(:,k)+D*u(k);
    y(k) = C*x(:,k)+D*u(k);
end
figure
subplot(211),stairs(t,u,'k-','linewi',2),grid on
title('control input')
subplot(212),plot(t,y,'k.',t,yhat,'g.','markersize',12),grid on
title('output')
legend('y','yhat')

%plot error
figure
plot(t,xhat(2,:)-x(2,:),'linewi',1.25)
grid on
hold on
plot(t,xhat(1,:)-x(1,:),'linewi',1.25)
title('state estimation error as a function of time')
hold on











