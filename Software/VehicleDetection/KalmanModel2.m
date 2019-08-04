
clear all
clc

x0=1;
y0=1;
vx0=-0.1;
vy0=-0.1;


x=zeros(1,11);
y=x;
vx=x;
vy=x;

x(1)=x0;
y(1)=y0;
vx(1)=vx0;
vy(1)=vy0;

Ts=1;

for i=2:11
    
    x(i)=x(i-1)+vx(i-1)*Ts;
    vx(i)=vx(i-1);
    y(i)=y(i-1)+y(i-1)*vx(i-1)/x(i-1)*Ts;
    vy(i)=y(i-1)*vx(i-1)/x(i-1);
    
end

plot(x,y)
xlabel('x')
ylabel('y')


x_predicted