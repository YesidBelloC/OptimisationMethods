
% %######################################################################
% %################    Define Sizes Here       ##########################
% %######################################################################
dimx=2;         % number of states
dimu=1;         % number of inputs
dimec=0;        % number of equality constraints
dimic=6;        % number of inequality constraints
N=20;           % length of horizon

% %######################################################################
% %##################    Define Parameters Here       ###################
% %######################################################################
%_____________________________Fixed Parameters___________________________
Xmax=30;
Xmin=0;
Vmax=33;
Vmin=0;
Tmax=5;
Tmin=-5;
%________________________________________________________________________
%__________________________Time Varying Parameters_______________________
syms tau dt          % Required (Do Not Change!)
Xf = sym('Xf',[dimx,1]);
Q  = sym('q',[dimx,1]);
W  = sym('w',[dimu,1]);
%--------------------Frozen Time-Varying Parameters----------------------
TVP_f=[Xf; Q; W; dt];  % "dt" MUST be the last Frozen TVP
%--------------------Dynamic Time-Vaying Parameters----------------------
TVP=[];
%____________________Exterior Penalty Costs Weights______________________
R_value=[1;1;1;1;1;1];
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
fxu=[Xk(2)
     Uk(1)];
%__________________________Constraints Definition__________________________
%___________________________in terms of Uk, Xk_____________________________
Gxu=[0];                      % Equality Constraints
Cxu=[Xmin-Xk(1)
     Xk(1)-Xmax             % Inequality Constraints
     Vmin-Xk(2)
     Xk(2)-Vmax
     Tmin-Uk(1)
     Uk(1)-Tmax];
%_______________________Objective Function Definition____________________
%___________________________in terms of Uk, Xk __________________________
Lk =0.5*(Xk-Xf)'*diag(Q)*(Xk-Xf)+0.5*Uk'*diag(W)*Uk; % Tranjectory Cost (Integral Terms)
Lk =Lk+R'*Cxu.^2;              % Required (Do Not Change!)
Phi=0;                         % Terminal Cost
shooting='single';
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
