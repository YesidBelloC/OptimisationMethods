function y = CxU(x,TVP,TVP_f,u,tau)
Xf1=TVP_f(1);
Xf2=TVP_f(2);
Xf3=TVP_f(3);
q1=TVP_f(4);
q2=TVP_f(5);
q3=TVP_f(6);
w1=TVP_f(7);
w11=TVP_f(8);
Op1=TVP_f(9);
dt=TVP_f(10);
Xk1=x(1);
Xk2=x(2);
Xk3=x(3);
Uk1=u(1);
y=zeros(1,1);
y(1)=Uk1 - 220;
