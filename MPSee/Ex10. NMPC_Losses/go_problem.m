
%% %%######################################################################
%% %%##################    Define Optimal Control Here       ##############
%% %%######################################################################
%% Uk is the vector of inputs
%% Xk is the vector of states
%%_______________________Vector Field Definition____________________________
%%_________________________in terms of Uk, Xk_______________________________
Fz   =(M*g*cos(Op(1)));
%%Miur =Miur0+Miur1*Vx^2;
Froll =Miur0*Fz;
Faer  =0.5*pair*Cair*Surf*Xk(2)^2; %%Asumo V viento..cero
Fw    =M*g*sin(Op(1));

RPM=Xk(2)/(3.6*((2*3.14)/60)*Rwr);
eff_k=(95.7926 - 1.3149e-4*(RPM-157.6525)^2 - 0.0013*(Uk(1)-73.2008)^2);
eff_k1=(95.7926 - 1.3149e-4*(abs(RPM)-157.6525)^2 - 0.0013*(abs(Uk(1))-73.2008)^2);

fxu=[Xk(2)
    ((Uk(1)/Rwr)-Froll-Faer-Fw)/M
    (Uk(1)/Rwr)*Xk(2)/eff_k1];
%%__________________________Constraints Definition__________________________
%%___________________________in terms of Uk, Xk_____________________________
Gxu=[0];                       %% Equality Constraints
Cxu=[Uk(1)-Tmax];
%% Cxu=[Uk(1)-Tmax
%%      Tmin-Uk(1)
%%      Xk(3)-Ebat
%%      Vmin-Xk(2)
%%      Xk(2)-Vmax];
%%_______________________Objective Function Definition____________________
%%___________________________in terms of Uk, Xk __________________________

Lk =0.5*(Xk-Xf)'*diag(Q)*(Xk-Xf) + 0.5*Uk'*diag(W)*Uk - W1*eff_k;%% Tranjectory Cost (Integral Terms)
%%Lk = -1*eff_k;               %% Tranjectory Cost (Integral Terms)
Lk =Lk+R'*Cxu.^2;              %% Required (Do Not Change!)
%%Phi=0;                         %% Terminal Cost
Phi=W1*(Xk(3)-Ebat);   %% Terminal Cost NOTA... X4 DA IGUAL
%%Phi=5e-3*W1*(Xk(3)-Ebat+Xk(4)^2);   %% Terminal Cost NOTA... X4 DA IGUAL%