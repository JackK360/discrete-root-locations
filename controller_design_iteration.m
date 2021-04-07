clear ; clc;
%% root locus technique, automation
fs = 5; %sampling frequency in Hz
T=1/fs; %sampling period in seconds
G=tf(0.01758*[1,0.876],poly([1,0.6703]),T);
zstar = 0.5158 + 0.4281i; % desired closed loop poles

%using the angle criterion
angleG = angle(evalfr(G,zstar)); %angle of plant
angleD = pi-angleG; %necessary angle of control

%making range of possible angles of zero
numControllers = 10; 
minAnglez1=angleD;
maxAnglez1=pi;
anglez1=linspace(minAnglez1+10*pi/180,maxAnglez1-10*pi/180,numControllers);
anglep1= anglez1 - angleD

%let's determine the controller!
for n = 1:numControllers
    %calculation location of zero
    z1(n) = real(zstar) - imag(zstar)/tan(anglez1(n));
    %calculation location of pole
    p1(n)=real(zstar)-imag(zstar)/tan(anglep1(n));
    
    %calculating controller
    DonK(n) =tf([1,-z1(n)],[1,-p1(n)],T);
    
    %using magnitude criterion to get K
    K(n) = 1/abs(evalfr(G*DonK(n),zstar));
    
    %put together the controller
    D(n) = K(n)*DonK(n);
    
    %caluclate the closed-loop poles
    G_CL(n) = feedback(G*D(n),1);
    CL_poles(:,n) = pole(G_CL(n));
    
    
end

%step response
ts = 2;
t=(0:T:2*ts)
y=step(G_CL(2),t)

figure
plot(t,y,'k.',t,ones(size(t)),'r-','markersize',16)
xlabel('t [s]'),ylabel('response')


