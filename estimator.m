clear all
clc
T=0.0126
%state-space model in modal coordinates
A = [0.9798 0.1232; -.1232 .9798];
B = [0.0127 ; 0.0375];
C=[.0306 -.0083];
D=[0];

Nmatrix = [C 0; A-eye(2) B]\[1;zeros(2,1)];
Nx = Nmatrix(1:2);
Nu = Nmatrix(3);

r = 0.5;
xr = Nx*r
uss = Nu*r;

%controller
%augmented model
zstar = [.9487+.0652i, .9487-.0652i]
K=acker(A,B,zstar)


t=(0:T:1.5)';
N=length(t);
x=zeros(2,N);
xI=zeros(1,N)
y=zeros(1,N);
u=zeros(1,N);
r=.5*ones(1,N);
xr=Nx*r;
uss=Nu*r;

u(1) = -K*(x(:,1)-xr(:,1));
for k =2:N
    x(:,k) = A*x(:,k-1) + B*u(k-1);
    y(k) = C*x(:,k) + D*u(k);
    u(k) = -K*(x(:,k)-xr(:,k));
end
figure
plot(t,r,'r-',t,y,'k.','markersize',14)
xlabel('t [s]'),legend('reference position','output position')
%%getting the state feedback matrix
A_aug=[A,zeros(2,1);-C,1];
B_aug=[B;-D]
zstar_aug=[zstar, .9*abs(zstar(1))];
K_aug=acker(A_aug,B_aug,zstar_aug);
eig(A_aug-B_aug*K_aug)
%define estimator gain matrix
betastar=[.8225+.2501i; .8225-.2501i]
L=acker(A',C',betastar)';
eig(A-L*C);



%redefine variables
x=zeros(2,N);
xI=zeros(1,N);
y=zeros(1,N);
u=zeros(1,N);
e=zeros(1,N);
xhat=zeros(2,N); %estimated states
yhat=zeros(1,N);

r=.5*ones(1,N);
xr=Nx*r;
uss=Nu*r

u(1)=-K_aug*([xhat(:,1);xI(1)]-[xr(:,1);0]);
e(1)=r(1)-y(1)
for k = 2:N
    x(:,k) = A*x(:,k-1)+B*u(k-1);
    xhat(:,k)=A*xhat(:,k-1)+B*u(k-1)-L*(yhat(k-1)-y(k-1));
    xI(k) = xI(k-1)+e(k-1);
    y(k)=C*x(:,k)+D*u(k);
    yhat(k)=C*xhat(:,k)+D*u(k)
    e(k)=r(k)-y(k);
    u(k)=-K_aug*([xhat(:,k);xI(k)]-[xr(:,k);0]);
end

figure
plot(t,r,'r-',t,y,'k.','markersize',14)
hold on
plot(t,yhat,'go','markersize',14)
xlabel('t [s]'),legend('reference','output')




