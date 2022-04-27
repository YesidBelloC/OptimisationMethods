function [dU,U,Control] = NMPC(dU0,U0,x0,TVP,TVP_f)
coder.allowpcode('plain');
Kmax=10;
errtol=1.000000e-02;
dimic=1;
iteration_out=0;
dimu=1;
TVP=TVP';
N=length(TVP);
dt=TVP_f(end);
params=[errtol, Kmax];
s=0;
xk=x0;
Con=zeros(dimic,N);
for i=1:N
Uk=U0(dimu*(i-1)+1:dimu*i);
Xdot=f(xk,TVP,TVP_f,Uk,i);
xk=xk+dt*Xdot;
Con(:,i)=CxU(xk,TVP,TVP_f,Uk,i);
s=s+sum(Con(:,i)>0);
end
if s>0
f0= FxU(x0, TVP,TVP_f, U0,Con);
dU=fdgmres(f0, x0, TVP,TVP_f, U0, params,dU0,Con);
U=U0+dU;
else
f0= FxU(x0, TVP,TVP_f, U0);
dU=fdgmres(f0, x0, TVP,TVP_f, U0, params,dU0);
U=U0+dU;
end
for rep=1:iteration_out
s=0;
xk=x0;
Con=zeros(dimic,N);
for i=1:N
Uk=U(dimu*(i-1)+1:dimu*i);
Xdot=f(xk,TVP,TVP_f,Uk,i);
xk=xk+dt*Xdot;
Con(:,i)=CxU(xk,TVP,TVP_f,Uk,i);
s=s+sum(Con(:,i)>0);
end
if s>0
f0= FxU(x0, TVP,TVP_f, U,Con);
dU=fdgmres(f0,x0,TVP,TVP_f,U,params,dU,Con);
U=U+dU;
else
f0= FxU(x0,TVP,TVP_f,U);
dU=fdgmres(f0,x0,TVP,TVP_f,U,params,dU);
U=U+dU;
end
end
Control=U(1:dimu);
