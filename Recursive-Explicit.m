%% Recursive solution to Differential equation
k = (0:1:19)';
z=zeros(20,1);
z(1)=1;
for i = 2:20
    z(i) = 0.5*z(i-1);
end
z2=1*(.5).^k;
figure
plot(k/5,z,'ko',k/5,z2,'r.'),grid on
xlabel('t(s)'),ylabel('z'),legend('recursive','explicit')
