
%% %%######################################################################
%% %%##################    Define Parameters Here       ###################
%% %%######################################################################
%%_____________________________Fixed Parameters___________________________
Rwr    = 0.580;
pair   = 0.9643;
Cair   = 0.4;
Surf   = 0.94243;
M      = 288; 
g      = 9.8;
Miur0  =0.01; 
Miur1  =7e-6;
%%Op     =0;
%%________________________________________________________________________
%%__________________________Time Varying Parameters_______________________
syms tau dt          %% Required (Do Not Change!)
%%syms Op
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
Emax=450000;%%75*45*3600;
Ebat=75*45*3600;
%%--------------------Frozen Time-Varying Parameters----------------------
TVP_f=[Xf; Q; W; W1; Op; dt];  %% "dt" MUST be the last Frozen TVP
%%--------------------Dynamic Time-Vaying Parameters----------------------
TVP=[];
%%____________________Exterior Penalty Costs Weights______________________
R_value=[1];
