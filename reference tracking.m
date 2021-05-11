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
