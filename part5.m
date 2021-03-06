clear all
clc
T=1/20
%state-space model in modal coordinates
A = [.5134];
B = [1];
C=[1.796];
D=[0];

Nmatrix = [C 0; A-eye(1) B]\[1;zeros(1,1)];
Nx = Nmatrix(1);
Nu = Nmatrix(2);

r = 4*pi;
xr = Nx*r
uss = Nu*r;

%controller
%augmented model
zstar = [.68+.25i,.68-.25i]
A_aug=[.5134 0; -1.796 1];
B_aug=[1;0]

K=acker(A_aug,B_aug,zstar)


t=(0:T:2.5)';
N=length(t);
x=zeros(2,N);
xI=zeros(1,N)
y=zeros(1,N);
u=zeros(1,N);
r=4*pi*ones(1,N);
xr=Nx*r;
uss=Nu*r;

%%getting the state feedback matrix
A_aug=[.5134 0; -1.796 1];
B_aug=[1;0]
zstar_aug=[zstar, .9*abs(zstar(1))];
K_aug=acker(A_aug,B_aug,zstar);
eig(A_aug-B_aug*K_aug)
%define estimator gain matrix
betastar=[.3]
L=.1188



%redefine variables
x=zeros(1,N);
xI=zeros(1,N);
y=zeros(1,N);
u=zeros(1,N);
e=zeros(1,N);
xhat=zeros(1,N); %estimated states
yhat=zeros(1,N);

r=4*pi*ones(1,N);
xr=Nx*r;
uss=Nu*r

u(1)=-K_aug*([xhat(1);xI(1)]-[xr(1);0]);
e(1)=r(1)-y(1)
for k = 2:N
    x(:,k) = A*x(:,k-1)+B*u(k-1);
    xhat(:,k)=A*xhat(:,k-1)+B*u(k-1)-L*(yhat(k-1)-y(k-1));
    xI(k) = xI(k-1)+e(k-1);
    y(k)=C*x(:,k)+D*u(k);
    yhat(k)=C*xhat(:,k)+D*u(k)+.05*randn(1)
    e(k)=r(k)-y(k);
    u(k)=-K_aug*([xhat(:,k);xI(k)]-[xr(:,k);0]);
end

figure
plot(t,r,'r-',t,y,'k.','markersize',14)
hold on
plot(t,yhat,'go','markersize',14)
xlabel('t [s]'),legend('reference','output','estimator')
ylabel('velocity [rad/s]')
figure
plot(t,u,'ko')
xlabel('t [s]')
ylabel('u [v]')
figure
plot(t,xhat-x,'ko')
title('state estimation error')
xlabel('time [s]')
ylabel('error')






