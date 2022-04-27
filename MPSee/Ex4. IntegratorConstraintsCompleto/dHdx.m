function y = dHdx(x,TVP,TVP_f,u,tau,lmd,Con)
Xf1=TVP_f(1);
Xf2=TVP_f(2);
q1=TVP_f(3);
q2=TVP_f(4);
w1=TVP_f(5);
dt=TVP_f(6);
Xk1=x(1);
Xk2=x(2);
Uk1=u(1);
Lmdk1=lmd(1);
Lmdk2=lmd(2);
y=zeros(2,1);
if nargin == 7
    con=Con(:,tau);
    if con(1)>0
        r1=1;
    else
        r1=0;
    end
    if con(2)>0
        r2=1;
    else
        r2=0;
    end
    if con(3)>0
        r3=1;
    else
        r3=0;
    end
    if con(4)>0
        r4=1;
    else
        r4=0;
    end
    if con(5)>0
        r5=1;
    else
        r5=0;
    end
    if con(6)>0
        r6=1;
    else
        r6=0;
    end
else
    r1=0;
    r2=0;
    r3=0;
    r4=0;
    r5=0;
    r6=0;
end
y(1)=conj(r2)*(2*Xk1 - 60) - (q1*(Xf1 - Xk1))/2 - q1*(conj(Xf1)/2 - conj(Xk1)/2) + 2*Xk1*conj(r1);
y(2)=Lmdk1 - (q2*(Xf2 - Xk2))/2 + conj(r4)*(2*Xk2 - 66) - q2*(conj(Xf2)/2 - conj(Xk2)/2) + 2*Xk2*conj(r3);
