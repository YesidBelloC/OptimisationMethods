
% %######################################################################
% %################    Define Sizes Here       ##########################
% %######################################################################
dimx=3;         % number of states
dimu=1;         % number of inputs
dimec=0;        % number of equality constraints
dimic=1;        % number of inequality constraints
N=10;            % length of horizon

% %######################################################################
% %##################    Define Parameters Here       ###################
% %######################################################################
%_____________________________Fixed Parameters___________________________
Rwr    = 0.580;
pair   = 0.9643;
Cair   = 0.4;
Surf   = 0.94243;
M      = 288; 
g      = 9.8;
Miur0  =0.01; 
Miur1  =7e-6;
%Op     =0;
%________________________________________________________________________
%__________________________Time Varying Parameters_______________________
syms tau dt          % Required (Do Not Change!)
%syms Op
Xf = sym('Xf',[dimx,1]);
Q  = sym('q',[dimx,1]);
W  = sym('w',[dimu,1]);
W1 = sym('w1',[dimu,1]);
Op = sym('Op',[dimu,1]);

Xmax=7000;
Xmin=0;
Vmax=33;
Vmin=0;
Tmax=220;
Tmin=-220;
Emax=450000;%75*45*3600;
Ebat=75*45*3600;
%--------------------Frozen Time-Varying Parameters----------------------
TVP_f=[Xf; Q; W; W1; Op; dt];  % "dt" MUST be the last Frozen TVP
%--------------------Dynamic Time-Vaying Parameters----------------------
TVP=[];
%____________________Exterior Penalty Costs Weights______________________
R_value=[1];

% %######################################################################
% %################    Do Not Change!       #############################
% %######################################################################
if dimic==0
    R=0;
else
    R=sym('r',[dimic,1]); 
end
Xk=sym('Xk',[dimx,1]);
Uk=sym('Uk',[dimu,1]);
Lmdk=sym('Lmdk',[dimx,1]);
if dimec==0
    Muk=0;
else
    Muk=sym('Muk',[dimec,1]);
end

% %######################################################################
% %##################    Define Optimal Control Here       ##############
% %######################################################################
% Uk is the vector of inputs
% Xk is the vector of states
%_______________________Vector Field Definition____________________________
%_________________________in terms of Uk, Xk_______________________________
Fz   =(M*g*cos(Op(1)));
%Miur =Miur0+Miur1*Vx^2;
Froll =Miur0*Fz;
Faer  =0.5*pair*Cair*Surf*Xk(2)^2; %Asumo V viento..cero
Fw    =M*g*sin(Op(1));

RPM=Xk(2)/(3.6*((2*3.14)/60)*Rwr);
eff_k=(95.7926 - 1.3149e-4*(RPM-157.6525)^2 - 0.0013*(Uk(1)-73.2008)^2);
eff_k1=(95.7926 - 1.3149e-4*(abs(RPM)-157.6525)^2 - 0.0013*(abs(Uk(1))-73.2008)^2);

fxu=[Xk(2)
    ((Uk(1)/Rwr)-Froll-Faer-Fw)/M
    (Uk(1)/Rwr)*Xk(2)/eff_k1];
%__________________________Constraints Definition__________________________
%___________________________in terms of Uk, Xk_____________________________
Gxu=[0];                       % Equality Constraints
Cxu=[Uk(1)-Tmax];
% Cxu=[Uk(1)-Tmax
%      Tmin-Uk(1)
%      Xk(3)-Ebat
%      Vmin-Xk(2)
%      Xk(2)-Vmax];
%_______________________Objective Function Definition____________________
%___________________________in terms of Uk, Xk __________________________

Lk =0.5*(Xk-Xf)'*diag(Q)*(Xk-Xf) + 0.5*Uk'*diag(W)*Uk - W1*eff_k;% Tranjectory Cost (Integral Terms)
%Lk = -1*eff_k;               % Tranjectory Cost (Integral Terms)
Lk =Lk+R'*Cxu.^2;              % Required (Do Not Change!)
%Phi=0;                         % Terminal Cost
Phi=W1*(Xk(3)-Ebat);   % Terminal Cost NOTA... X4 DA IGUAL
%Phi=5e-3*W1*(Xk(3)-Ebat+Xk(4)^2);   % Terminal Cost NOTA... X4 DA IGUALshooting='single';
continuation='no';
Kmax=10;
errtol=0.01;
iter_out=1-1;
Usize=N*(dimu+dimec);
U0=0*ones(Usize,1);
assignin('base', 'U0', U0);
dU0=0*ones(Usize,1);
assignin('base', 'dU0', dU0);
Coder(shooting,continuation,N,dimx,dimu,dimec,dimic,TVP,TVP_f,Xk,Uk,Lmdk,Muk,fxu,Gxu,Cxu,Lk,Phi,R_value,Kmax,errtol,iter_out);
