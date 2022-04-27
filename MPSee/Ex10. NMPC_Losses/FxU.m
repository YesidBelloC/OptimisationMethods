function Fvector = FxU(Xvector,TVP,TVP_f,Uvector,con)
N=10;
dimu=1;
dimec=0;
dimic=1;
dimx=3;
coder.allowpcode('plain');
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
Fvector=zeros(10,1);
X=zeros(dimx,N+1);
Lmd=zeros(dimx,N+1);
lmdN=zeros(dimx,1);
X(:,1)=Xvector;
for i=2:N+1
    Xk=X(:,i-1);
    Uk=Uvector((i-2)*(dimec+dimu)+1:(i-1)*(dimec+dimu));
    tau=(i-1);
    X(:,i)=X(:,i-1)+dt*f(Xk,TVP,TVP_f,Uk,tau);
end
Xk1=X(1,N+1);
Xk2=X(2,N+1);
Xk3=X(3,N+1);
lmdN(1)=0;
lmdN(2)=0;
lmdN(3)=w11;
Lmd(:,N+1)=lmdN';
if nargin == 5
    for i=N:-1:1
        Xk=X(:,i);
        Uk=Uvector((i-1)*(dimec+dimu)+1:(i)*(dimec+dimu));
        tau=i;
        Lmd(:,i)=Lmd(:,i+1)+dt*dHdx(Xk,TVP,TVP_f,Uk,tau,Lmd(:,i+1), con);
    end
    for i=1:N
        Xk=X(:,i);
        Uk=Uvector((i-1)*(dimec+dimu)+1:(i)*(dimec+dimu));
        Lmdk=Lmd(:,i+1);
        tau=i;
        Fvector((i-1)*dimu+1:(i)*(dimu),:)=dHdu(Xk,TVP,TVP_f,Uk,tau,Lmdk,con);
    end
else
    for i=N:-1:1
        Xk=X(:,i);
        Uk=Uvector((i-1)*(dimec+dimu)+1:(i)*(dimec+dimu));
        tau=i;
        Lmd(:,i)=Lmd(:,i+1)+dt*dHdx(Xk,TVP,TVP_f,Uk,tau,Lmd(:,i+1));
    end
    for i=1:N
        Xk=X(:,i);
        Uk=Uvector((i-1)*(dimec+dimu)+1:(i)*(dimec+dimu));
        Lmdk=Lmd(:,i+1);
        tau=i;
        Fvector((i-1)*dimu+1:(i)*(dimu),:)=dHdu(Xk,TVP,TVP_f,Uk,tau,Lmdk);
    end
end
