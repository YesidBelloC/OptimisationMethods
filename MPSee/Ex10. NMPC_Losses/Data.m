%Parametros ctes Moto

    Rwr    = 0.3302; %[m]
    pair   = 1.25;
    Cair   = 0.3;
    Surf   = 0.94243;
    M      = 288;%288; %150; Peso con Biker y Carga
    g      = 9.8;
    Miur0  = 0.01; %[0.01 0.008]Very good concrete
    Miur1  = 7e-6;
    
%Data Pedals:

    Ja    = 0.7282; %Da(1.5)>=2*sqrt(Ta*Ka(2))
    Kre_a = 0.10;
    Ktr   = 4.4; %220Nm/50°
    Jb    = 0.7282;
    Kre_b = 0.12;
    Kb    = 9.6; %480Nm/50°
    Tb_a  = 20;  %cte; Kba*v con Kba=cte o Kba=

% v_test=linspace(1,33);
% %B_test=12*v_test;
% B_test=sqrt(140*v_test);
% plot(v_test,B_test)

%Model:

%X1=Theta
%X2=Thetap

%Ja*X2p_a=-Kre_a*X1 + Ta + Ta_h
%Ttr=Ktr*X1 -> ref X1
%Jb*X2p_b=-Kre_b*X1 + Tb + Tb_h
%Ttr=Kb*X1+f(v) -> ref X1

%% Losses and Thermal Model

Udc = 72;        % Tension de la Baterie[v]
Ns=ceil(72/12.6); %Vol_Bat/Vol_CeldaMedida
Np=ceil(45/6.650); %Vol_Bat/Vol_CeldaMedida
Cap  = 45*3600; %[As=Ah*3600]

%% Param Thermal Circuit Losses

% Inverteur
Td0=25;
Tl0=25;
Tai0=25;
Tc0=25;
Cd   = 0.4*0.996;     %mCp [J/C] ..  [g]*[J/(g°C)] Aluminium + ceramic
Cl   = 560*0.896;     %mCp [J/C] ..  [g]*[J/(g°C)] Aluminium 
Cc   = 43667.5*0.896; %mCp [J/C] .. [g]*[J/(g°C)] Aluminium 
Cai  = 3.3*0.001;     %mCp [J/C] ..  [g]*[J/(g°C)] Dry Air 
%Dimensiones Lat 0.283*0.146*0.005 283x146x5mm.
%Dimensiones Caj 0.283*0.146*0.062 283x146x62mm.
%Dimensiones Mos 0.015*0.001*0.010 15x1x10mm.
Adl = 0.015*0.010*2;
Alai= 0.283*0.146-0.015*0.01*6;
Alc = 0.283*0.146;
Aaic= 0.283*0.146+0.146*0.062*2+0.283*0.062*2;
Aca = (0.283*0.146*2+0.146*0.062*2+0.283*0.062*2);
%Coef de Intercambio de Temp
h_air  =0.026; %[w/(m*C)]
k_alum =209.3; %[w/(m*C)]
k_cop  =401;   %[w/(m*C)]
k_hier =80.2;  %[w/(m*C)]
km_Crem=800; %[w/(m*C)]
Rdl  = 1/(Adl*(k_alum*km_Crem));  %1/hA o 1/kA [K/w]
Rlai = 1/(Alai*k_alum);  %1/hA o 1/kA [K/w]
Rail = 1/(Alai*h_air);  %1/hA o 1/kA [K/w]
Rlc  = 1/(Alc*k_alum); %1/hA o 1/kA [K/w]
Raic = 1/(Aaic*h_air); %1/hA o 1/kA [K/w]
Rcai = 1/(Aaic*k_alum); %1/hA o 1/kA [K/w]
Rca  = 1/(Aca*k_alum);  %1/hA o 1/kA [K/w]

%Battery
Tb0=25;
Ti0=25;
To0=25;
Cb = 14000*3.56; %mCp [J/C] .. [g]*[J/(g°C)] Lithium
Co = 3000*0.921; %mCp [J/C] .. [g]*[J/(g°C)] Aluminium
Ci = 5*0.001;  %mCp [J/C] ..   [g]*[J/(g°C)] Air 25° -> Mas aire... 1.2Kg = 1m3
%Dimensiones Bat           0.18*0.20*0.22
%Dimensiones Caj           0.20*0.24*0.26
%Dimensiones Air Caj- Bat :0.20*0.24*0.26 - 0.18*0.20*0.22
Abi = 0.18*0.20+0.18*0.22+0.20*0.22;
Abo = 0.18*0.20+0.18*0.22+0.20*0.22;
Aba = 0.18*0.20;
Aio = (0.20*0.24+ 0.20*0.26 + 0.24*0.26)-Abo;
Aoa = 0.20*0.24+ 0.20*0.26 + 0.24*0.26;
%Coef de Intercambio de Temp
h_air  =0.026;
k_alum =209.3;
k_litio=301.2;
Rbi = 1/(Abi*k_litio);   %1/hA o 1/kA [K/w]
Rib = 1/(Abi*h_air);     %1/hA o 1/kA [K/w]
Rbo = 1/(Abo*k_litio);   %1/hA o 1/kA [K/w]
Rob = 1/(Abo*k_alum);    %1/hA o 1/kA [K/w]
Rba = 1/(Aba*k_litio);   %1/hA o 1/kA [K/w]
Rab = 1/(Aba*h_air);     %1/hA o 1/kA [K/w]
Rio = 1/(Aio*h_air);     %1/hA o 1/kA [K/w]
Roi = 1/(Aio*k_alum);    %1/hA o 1/kA [K/w]
Roa = 1/(Aoa*k_alum);    %1/hA o 1/kA [K/w]
e = 0.09;  % emisividad del cromo negro
A = 0.25;  % Area a la que le da el sol. 0.5*0.5 [m]
GDd=1353;
GD = GDd*0.6; %Radiacion directa ... la saco de la radiacion total estimada NASA
Gd = GD*0.1;  %Radiacion indirecta
as = 0.87;    %absortividad solar del cromo negro
ang= deg2rad(60); %Angulo entre la normal de la sup del vehicle y el sol.
Qrd = as*(GD*cos(ang)+Gd); %Transferencia de calor por radiaoctividad, la emision 
                           %   de calor absorvida y emitida por fases nobles es despreciable... 
                           %   en agua o co2 es alta.
                           
%% Plot de limitacion de angulo
syms xpp Ut th vel
%xpp=Ut/(Rwr*M)-M*g*sin(th)/M-M*g*cos(th)*Miur0/M-pair*Cair*Surf*vel^2/M
Ut=(xpp+M*g*sin(th)/M+M*g*cos(th)*Miur0/M+pair*Cair*Surf*vel^2/M)*(Rwr*M)

