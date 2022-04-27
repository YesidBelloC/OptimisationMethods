function [problem,guess] = DoubleIntegratorNMPC
%https://www.qsmotor.com/product/17inch-4000w-scooter-motor/
%BangBang - BangBang Control (Double Integrator Minimum Time Repositioning) Problem

% Plant model name, used for Adigator
problem.data.plantmodel = 'EffTfOPCPlant';

% Initial time. t0<tf
problem.time.t0=0;

% Final time. Let tf_min=tf_max if tf is fixed.
problem.time.tf_min=600;     
problem.time.tf_max=600; 
guess.tf=600;

% Parameters bounds. pl=< p <=pu
problem.parameters.pl=[];
problem.parameters.pu=[];
guess.parameters=[];

% Initial conditions for system
problem.states.x0=[0 0];

% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
problem.states.x0l=[0 0]; 
problem.states.x0u=[0 0]; 

% State bounds. xl=< x <=xu
problem.states.xl=[0 -10];
problem.states.xu=[inf 30];

% State error bounds
problem.states.xErrorTol=[1e-3 1e-3];

% State constraint error bounds
problem.states.xConstraintTol=[1e-3 1e-3];

% Terminal state bounds. xfl=< xf <=xfu
problem.states.xfl=[0 0]; 
problem.states.xfu=[inf 30];

% Guess the state trajectories with [x0 xf]
guess.states(:,1)=[0 1000];
guess.states(:,2)=[0 30];

% Number of control actions N 
% Set problem.inputs.N=0 if N is equal to the number of integration steps.  
% Note that the number of integration steps defined in settings.m has to be divisible 
% by the  number of control actions N whenever it is not zero.
problem.inputs.N=0;

% Input bounds
problem.inputs.ul=[-220];
problem.inputs.uu=[220];

% Bounds on first control action
problem.inputs.u0l=[-220];
problem.inputs.u0u=[220]; 

% bounds on the rate of change for control variables (optional) 
problem.inputs.url=[-1e-2]; 
problem.inputs.uru=[1e-2]; 

% Input constraint error bounds
problem.inputs.uConstraintTol=[1e-2];
problem.inputs.urConstraintTol=[1e-2]; 

% Guess the input sequences with [u0 uf]
guess.inputs(:,1)=[220 -220];

% Choose the set-points if required
problem.setpoints.states=[];
problem.setpoints.inputs=[];

% Bounds for path constraint function gl =< g(x,u,p,t) =< gu
 problem.constraints.gl=[0];
 problem.constraints.gu=[inf];
 problem.constraints.gTol=[0.5];
% problem.constraints.gl=[];
% problem.constraints.gu=[];
% problem.constraints.gTol=[];

% Bounds for boundary constraints bl =< b(x0,xf,u0,uf,p,t0,tf) =< bu
problem.constraints.bl=[];
problem.constraints.bu=[];

% store the necessary problem parameters used in the functions
load ProfilDistVelAlturTiem1.mat
Speed=V_SPEEDetDISTANCEetHAUTEUR(:,1);
auxdata.Speed=Speed;

[S,D]=Trajectory();
auxdata.S_angle=S;
auxdata.D_distance=D;
problem.data.auxdata=auxdata;

% Get function handles and return to Main.m
problem.functions={@L,@E,@f,@g,@avrc,@b};
problem.functions_unscaled={@L_unscaled,@E_unscaled,@f_unscaled,@g_unscaled,@avrc_unscaled,@b_unscaled};
problem.constraintErrorTol=[problem.constraints.gTol,problem.constraints.gTol,problem.states.xConstraintTol,problem.states.xConstraintTol,problem.inputs.uConstraintTol,problem.inputs.uConstraintTol];
%------------- END OF CODE --------------



function stageCost=L_unscaled(x,xr,u,ur,p,t,vdat)
% L_unscaled - Returns the stage cost.
% The function must be vectorized and
% xi, ui are column vectors taken as x(:,i) and u(:,i) (i denotes the i-th
% variable)

% Syntax:  stageCost = L(x,xr,u,ur,p,t,data)
% Inputs:
%    x  - state vector
%    xr - state reference
%    u  - input
%    ur - input reference
%    p  - parameter
%    t  - time
%    data- structured variable containing the values of additional data used inside
%          the function
% Output:
%    stageCost - Scalar or vectorized stage cost
%  Remark: If the stagecost does not depend on variables it is necessary to multiply
%          the assigned value by t in order to have right vector dimesion when called for the optimization. 
%          Example: stageCost = 0*t;
%------------- BEGIN CODE --------------

auxdata = vdat.auxdata;
Speed=auxdata.Speed;

X  = x(:,1);
Vx = x(:,2);
T  = u(:,1);

Speed_V=[];
cte_offset=0;
for i=1:size(t,1)
    Speed_V=[Speed_V; Speed(ceil(t(i)+0.1))/3.6+cte_offset];
end

%e1=x(:,2)-Speed_V;
e1=x(:,1)-7000;
%u1=u(:,1);

RPM=Vx./(0.580*0.10472);
eff_k=(95.7926 - 1.3149e-4*(RPM-157.6525).^2 - 0.0013*(T-73.2008).^2);

stageCost = -0*eff_k + 1*e1.*e1;% + 5e-4*(u(:,1).*u(:,1));
%------------- END OF CODE --------------

function boundaryCost=E_unscaled(x0,xf,u0,uf,p,t0,tf,vdat) 
% E_unscaled - Returns the boundary value cost
% Syntax:  boundaryCost=E(x0,xf,u0,uf,p,tf,data)
% Inputs:
%    x0  - state at t=0
%    xf  - state at t=tf
%    u0  - input at t=0
%    uf  - input at t=tf
%    p   - parameter
%    tf  - final time
%    data- structured variable containing the values of additional data used inside
%          the function
% Output:
%    boundaryCost - Scalar boundary cost
%------------- BEGIN CODE --------------

boundaryCost=0;

%------------- END OF CODE --------------

function dx = f_unscaled(x,u,p,t,vdat)

% f_unscaled - Returns the ODE right hand side where x'= f(x,u,p,t)
% The function must be vectorized and
% xi, ui, pi are column vectors taken as x(:,i), u(:,i) and p(:,i). Each
% state corresponds to one column of dx.
% Syntax:  dx = f(x,u,p,t,data)
% Inputs:
%    x  - state vector
%    u  - input
%    p  - parameter
%    t  - time
%    data-structured variable containing the values of additional data used inside
%          the function 
% Output:
%    dx - time derivative of x
%  Remark: If the i-th ODE right hand side does not depend on variables it is necessary to multiply
%          the assigned value by a vector of ones with the same length  of t  in order 
%          to have  a vector with the right dimesion  when called for the optimization. 
%          Example: dx(:,i)= 0*ones(size(t,1)); 
%------------- BEGIN CODE --------------
auxdata = vdat.auxdata;
S=auxdata.S_angle;
D=auxdata.D_distance;

Rwr    = 0.580; %[m]
pair   = 0.9643;
Cair   = 0.4;
Supf   = 0.94243;
M      = 288; %150; Peso con Biker y Carga
g      = 9.8;
%Op     = 0.01;

X  = x(:,1);
Vx = x(:,2);
T  = u(:,1);

Op =[];
for i=1:size(X,1)
    Op_index = find(abs(D-X(i))==min(abs(D-X(i))),1);
    if abs(S(Op_index))<0.01
        Op = [Op S(Op_index)];
    else
        Op = [Op sign(S(Op_index))*0.01];
    end
end

%% Intermediates Equations:
Fzr   =(M*g*cos(Op));
Miur0=0.009; %[0.01 0.008]Very good concrete
Miur1=7e-6;
%Miur =Miur0+Miur1*Vx.^2;

%Froll =Miur*Fzr;
Froll =Miur0*Fzr;
Faer  =0.5*pair*Cair*Supf*Vx.^2; %Asumo V viento..cero
Fw    =M*g*sin(Op);

%% Compute dxdt
dx = x;
% x1 Xp
dx(:,1) = Vx;
% x2 Vxp
dx(:,2) = ((T/Rwr)-Froll'-Faer-Fw')/M;
%------------- END OF CODE --------------

function c=g_unscaled(x,u,p,t,vdat)
% g_unscaled - Returns the path constraint function where gl =< g(x,u,p,t) =< gu
% The function must be vectorized and
% xi, ui, pi are column vectors taken as x(:,i), u(:,i) and p(:,i). Each
% constraint corresponds to one column of c
% Syntax:  c=g(x,u,p,t,data)
%
% Inputs:
%    x  - state vector
%    u  - input
%    p  - parameter
%    t  - time
%   data- structured variable containing the values of additional data used inside
%          the function
% Output:
%    c - constraint function
%
%------------- BEGIN CODE --------------
% npos = x(:,2);
% epos = x(:,3);
% c=[(npos-data.auxdata.obs_npos).^2+(epos-data.auxdata.obs_epos).^2-data.auxdata.obs_r.^2];
%c=[];
X  = x(:,1);
Vx = x(:,2);
T  = u(:,1);
RPM=Vx./(0.580*0.10472);

%OPCION 1
c=3000*0.7*0.7*0.9-T.*Vx;

%OPCION 2 Computation Time.. LONG !!! VERY LONG!!
% c=[];
% for i=1:size(T,1)
%     if abs(T(i))>=98.28
%         c(i,1) = -((-1.5204)*Vx(i)/(0.580*0.10472)+359.78-(abs(T(i)))); 
%     elseif abs(T(i))<98.28 && abs(T(i))>=49.34
%         c(i,1) = -((-0.34198)*Vx(i)/(0.580*0.10472)+157.06-(abs(T(i)))); 
%     elseif abs(T(i))<49.34 && abs(T(i))>=15.1
%         c(i,1) = -((-0.1328)*Vx(i)/(0.580*0.10472)+91.24-(abs(T(i)))); 
%     elseif abs(T(i))<15.1
%         c(i,1) = -((-0.0539)*Vx(i)/(0.580*0.10472)+45.99-(abs(T(i)))); 
%     end
% end


%------------- END OF CODE --------------
function bc=b_unscaled(x0,xf,u0,uf,p,t0,tf,vdat,varargin)
% b_unscaled - Returns a column vector containing the evaluation of the boundary constraints: bl =< bf(x0,xf,u0,uf,p,t0,tf) =< bu
% Syntax:  bc=b(x0,xf,u0,uf,p,tf,data)
%
% Inputs:
%    x0  - state at t=0
%    xf  - state at t=tf
%    u0  - input at t=0
%    uf  - input at t=tf
%    p   - parameter
%    tf  - final time
%    data- structured variable containing the values of additional data used inside
%          the function
%
%          
% Output:
%    bc - column vector containing the evaluation of the boundary function 
%
%------------- BEGIN CODE --------------
varargin=varargin{1};
bc=[];
%------------- END OF CODE --------------
% When adpative time interval add constraint on time
%------------- BEGIN CODE --------------
if length(varargin)==2
    options=varargin{1};
    t_segment=varargin{2};
    if ((strcmp(options.transcription,'hpLGR')) || (strcmp(options.transcription,'globalLGR')))  && options.adaptseg==1 
        if size(t_segment,1)>size(t_segment,2)
            bc=[bc;diff(t_segment)];
        else
            bc=[bc;diff(t_segment)'];
        end
    end
end
%------------- END OF CODE --------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Leave the following unchanged! %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function stageCost=L(x,xr,u,ur,p,t,vdat)
% L - Returns the stage cost.
% Warp function
%------------- BEGIN CODE --------------
if isfield(vdat,'Xscale')
    x=scale_variables_back( x, vdat.Xscale, vdat.Xshift );
    if ~isempty(xr)
        xr=scale_variables_back( xr, vdat.Xscale, vdat.Xshift );
    end
    u=scale_variables_back( u, vdat.Uscale, vdat.Ushift );
    if isfield(vdat,'Pscale')
        p=scale_variables_back( p, vdat.Pscale, vdat.Pshift );
    end
    if ~isempty(ur)
        ur=scale_variables_back( ur, vdat.Uscale, vdat.Ushift );
    end
    if strcmp(vdat.mode.currentMode,'Feasibility')
        stageCost=0*t;
    else
        stageCost=L_unscaled(x,xr,u,ur,p,t,vdat);
    end
else
    if strcmp(vdat.mode.currentMode,'Feasibility')
        stageCost=0*t;
    else
        stageCost=L_unscaled(x,xr,u,ur,p,t,vdat);
    end
end
%------------- END OF CODE --------------

function boundaryCost=E(x0,xf,u0,uf,p,t0,tf,vdat) 
% E - Returns the boundary value cost
% Warp function
%------------- BEGIN CODE --------------
if isfield(vdat,'Xscale')
    x0=scale_variables_back( x0', vdat.Xscale, vdat.Xshift );
    xf=scale_variables_back( xf', vdat.Xscale, vdat.Xshift );
    u0=scale_variables_back( u0', vdat.Uscale, vdat.Ushift );
    uf=scale_variables_back( uf', vdat.Uscale, vdat.Ushift );
    if isfield(vdat,'Pscale')
        p=scale_variables_back( p', vdat.Pscale, vdat.Pshift );
    end
    if strcmp(vdat.mode.currentMode,'Feasibility')
        boundaryCost=sum(sum(p(:,end-vdat.mode.np*2+1:end)));
    else
        boundaryCost=E_unscaled(x0,xf,u0,uf,p,t0,tf,vdat);
    end
else
    if strcmp(vdat.mode.currentMode,'Feasibility')
        boundaryCost=sum(sum(p(:,end-vdat.mode.np*2+1:end)));
    else
        boundaryCost=E_unscaled(x0,xf,u0,uf,p,t0,tf,vdat);
    end
end
%------------- END OF CODE --------------


function dx = f(x,u,p,t,vdat)
% f - Returns the ODE right hand side where x'= f(x,u,p,t)
% Warp function
%------------- BEGIN CODE --------------
if isfield(vdat,'Xscale')
    x=scale_variables_back( x, vdat.Xscale, vdat.Xshift );
    u=scale_variables_back( u, vdat.Uscale, vdat.Ushift );
    if isfield(vdat,'Pscale')
        p=scale_variables_back( p, vdat.Pscale, vdat.Pshift );
    end
    dx = f_unscaled(x,u,p,t,vdat);
    dx= scale_variables( dx, vdat.Xscale, 0 );
else
    dx = f_unscaled(x,u,p,t,vdat);
end
%------------- END OF CODE --------------

function c=g(x,u,p,t,vdat)
% g - Returns the path constraint function where gl =< g(x,u,p,t) =< gu
% Warp function
%------------- BEGIN CODE --------------
if isfield(vdat,'Xscale')
    x=scale_variables_back( x, vdat.Xscale, vdat.Xshift );
    u=scale_variables_back( u, vdat.Uscale, vdat.Ushift );
    if isfield(vdat,'Pscale')
        p=scale_variables_back( p, vdat.Pscale, vdat.Pshift );
    end
    c = g_unscaled(x,u,p,t,vdat);
else
    c = g_unscaled(x,u,p,t,vdat);
end
if isfield(vdat,'gFilter')
    c(:,vdat.gFilter)=[];
end
if strcmp(vdat.mode.currentMode,'Feasibility')
    c=[c-p(:,end-vdat.mode.np*2+1:end-vdat.mode.np) c+p(:,end-vdat.mode.np+1:end)];
end
%------------- END OF CODE --------------

function cr=avrc(x,u,p,t,data)
% avrc - Returns the rate constraint algebraic function where [xrl url] =<
% avrc(x,u,p,t) =< [xru uru]
% The function must be vectorized and
% xi, ui, pi are column vectors taken as x(:,i), u(:,i) and p(:,i). Each
% constraint corresponds to one column of c
% Syntax:  cr=avrc(x,u,p,t,data)
% Inputs:
%    x  - state vector
%    u  - input
%    p  - parameter
%    t  - time
%   data- structured variable containing the values of additional data used inside
%          the function
% Output:
%    cr - constraint function
%------------- BEGIN CODE --------------
[ cr ] = addRateConstraint( x,u,p,t,data );
%------------- END OF CODE --------------

function bc=b(x0,xf,u0,uf,p,t0,tf,vdat,varargin)
% b - Returns a column vector containing the evaluation of the boundary constraints: bl =< bf(x0,xf,u0,uf,p,t0,tf) =< bu
% Warp function
%------------- BEGIN CODE --------------
bc=b_unscaled(x0,xf,u0,uf,p,t0,tf,vdat,varargin);
if isfield(vdat,'Xscale')
    if ~isempty(bc)
        x0=scale_variables_back( x0', vdat.Xscale, vdat.Xshift );
        xf=scale_variables_back( xf', vdat.Xscale, vdat.Xshift );
        u0=scale_variables_back( u0', vdat.Uscale, vdat.Ushift );
        uf=scale_variables_back( uf', vdat.Uscale, vdat.Ushift );
        if isfield(vdat,'Pscale')
            p=scale_variables_back( p', vdat.Pscale, vdat.Pshift );
        end
        bc=b_unscaled(x0,xf,u0,uf,p,t0,tf,vdat,varargin);
    end
end

%------------- END OF CODE ---------------------