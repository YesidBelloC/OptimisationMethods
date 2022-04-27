function y = dHdu(x,TVP,TVP_f,u,tau,lmd,Con)
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
Lmdk1=lmd(1);
Lmdk2=lmd(2);
Lmdk3=lmd(3);
if nargin == 7
    con=Con(:,tau);
    if con(1)>0
        r1=1;
    else
        r1=0;
    end
else
    r1=0;
end
y=zeros(1,1);
y(1)=(25*Lmdk2)/4176 + (Uk1*w1)/2 + conj(r1)*(2*Uk1 - 440) + (w1*conj(Uk1))/2 + w11*((13*Uk1)/5000 - 1189513/6250000) - (50*Lmdk3*conj(Xk2))/(29*((13*(abs(Uk1) - 91501/1250)^2)/10000 + (2425562378252069*((62500*abs(Xk2))/13659 - 63061/400)^2)/18446744073709551616 - 421300310219581/4398046511104)) + (13*Lmdk3*conj(Uk1)*conj(Xk2)*sign(Uk1)*(abs(Uk1) - 91501/1250))/(2900*((13*(abs(Uk1) - 91501/1250)^2)/10000 + (2425562378252069*((62500*abs(Xk2))/13659 - 63061/400)^2)/18446744073709551616 - 421300310219581/4398046511104)^2);
