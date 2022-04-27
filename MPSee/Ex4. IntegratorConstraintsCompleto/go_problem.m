
%% %%######################################################################
%% %%##################    Define Optimal Control Here       ##############
%% %%######################################################################
%% Uk is the vector of inputs
%% Xk is the vector of states
%%_______________________Vector Field Definition____________________________
%%_________________________in terms of Uk, Xk_______________________________
fxu=[Xk(2)
     Uk(1)];
%%__________________________Constraints Definition__________________________
%%___________________________in terms of Uk, Xk_____________________________
Gxu=[0];                      %% Equality Constraints
Cxu=[Xmin-Xk(1)
     Xk(1)-Xmax             %% Inequality Constraints
     Vmin-Xk(2)
     Xk(2)-Vmax
     Tmin-Uk(1)
     Uk(1)-Tmax];
%%_______________________Objective Function Definition____________________
%%___________________________in terms of Uk, Xk __________________________
Lk =0.5*(Xk-Xf)'*diag(Q)*(Xk-Xf)+0.5*Uk'*diag(W)*Uk; %% Tranjectory Cost (Integral Terms)
Lk =Lk+R'*Cxu.^2;              %% Required (Do Not Change!)
Phi=0;                         %% Terminal Cost
