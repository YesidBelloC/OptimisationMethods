
%% %%######################################################################
%% %%##################    Define Parameters Here       ###################
%% %%######################################################################
%%_____________________________Fixed Parameters___________________________
Xmax=7000;
Xmin=0;
Vmax=33;
Vmin=0;
Tmax=220;
Tmin=-220;
%%________________________________________________________________________
%%__________________________Time Varying Parameters_______________________
syms tau dt          %% Required (Do Not Change!)
Xf = sym('Xf',[dimx,1]);
Q  = sym('q',[dimx,1]);
W  = sym('w',[dimu,1]);
%%--------------------Frozen Time-Varying Parameters----------------------
TVP_f=[Xf; Q; W; dt];  %% "dt" MUST be the last Frozen TVP
%%--------------------Dynamic Time-Vaying Parameters----------------------
TVP=[];
%%____________________Exterior Penalty Costs Weights______________________
R_value=[1;1;1;1;1;1];