clear all
clc
T=1/20
%state-space model in modal coordinates
A = [.5134];
B = [1];
C=[1.796];
D=[0];

Nx = .5568
Nu = .2709
r = 4*pi;
xr = Nx*r
uss = Nu*r;

%controller
%augmented model
K=[0.1534   -0.0918]
%%%%%%
%%%%%%
%%%%%

t=(0:T:1.5)';
N=length(t);
x=zeros(2,N);
y=zeros(2,N);
u=zeros(1,N);
r=4*pi*ones(1,N);
xr=Nx*r;
uss=Nu*r;

u(1) = -K*(x(:,1)-xr(1));
for k =2:N
    x(:,k) = A*x(:,k-1) + B*u(k-1);
    y(:,k) = C*x(:,k) + D*u(k);
    u(k) = -K*(x(:,k)-xr(:,k));
end
figure
plot(t,r,'r-',t,y,'k.','markersize',14)
xlabel('t [s]'),legend('reference','output position')
figure
plot(t,u,'r-')
xlabel('t [s]'),legend('u')
