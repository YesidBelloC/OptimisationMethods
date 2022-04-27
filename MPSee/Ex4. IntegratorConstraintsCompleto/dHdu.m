function y = dHdu(x,TVP,TVP_f,u,tau,lmd,Con)
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
y=zeros(1,1);
y(1)=Lmdk2 + (Uk1*w1)/2 + conj(r5)*(2*Uk1 + 10) + conj(r6)*(2*Uk1 - 10) + (w1*conj(Uk1))/2;
