% Copyright (C) 2018 Yuanbo Nie, Omar Faqir, and Eric Kerrigan. All Rights Reserved.
% Imperial College London London  England, UK 
% ICLOCS (Imperial College London Optimal Control) Version 2.0
%--------------------------------------------------------

clear all;close all;format compact;
global sol;  
sol=[];                                     % Initialize solution structure

options= settings_Auto(40);                         % Get options and solver settings 
[problem,guess]=DoubleIntegrator;                   % Fetch the problem definition
errorHistory=zeros(2,length(problem.states.x0));
npsegmentHistory=zeros(2,1);
ConstraintErrorHistory=zeros(2,length(problem.constraintErrorTol));
timeHistory=zeros(1,2);
iterHistory=zeros(1,2);
solutionHistory=cell(1,2);

maxAbsError=1e9;
i=1; imax=2; % Si uso Vector de angulos... debo bajar a 2.. si ang es cte.. puedo subir

while (any(maxAbsError>problem.states.xErrorTol) || any(maxAbsConstraintError>problem.constraintErrorTol)) && i<=imax    
    [infoNLP,data,options]=transcribeOCP(problem,guess,options); % Format for NLP solver
    [solution,status,data] = solveNLP(infoNLP,data);             % Solve the NLP
    [solution]=output(problem,solution,options,data,4);          % Output solutions

    maxAbsError=max(abs(solution.Error));
    maxAbsConstraintError=max(solution.ConstraintError);
    errorHistory(i,:)=maxAbsError;
    iterHistory(i)=status.iter;
    ConstraintErrorHistory(i,:)=maxAbsConstraintError;
    timeHistory(i)=solution.computation_time;
    solutionHistory{i}=solution;
  
    if (any(maxAbsError>problem.states.xErrorTol) || any(maxAbsConstraintError>problem.constraintErrorTol)) && i<=imax
        [ options, guess ] = doMeshRefinement( options, problem, guess, data, solution, i );
    end
    i=i+1;
    disp(horzcat('Iteracion: ',num2str(i)))
end

MeshRefinementHistory.errorHistory=errorHistory;
MeshRefinementHistory.timeHistory=timeHistory;
MeshRefinementHistory.iterHistory=iterHistory;
MeshRefinementHistory.ConstraintErrorHistory=ConstraintErrorHistory;
%%

xx=linspace(solution.T(1,1),solution.tf,1000);

if (strcmp(options.transcription,'globalLGR')) || (strcmp(options.transcription,'hpLGR'))
    figure
    subplot(2,1,1)
    xlabel('Time [s]')
    ylabel('Position [m]')
    grid on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,1,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'r' )
    subplot(2,1,2)
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,2,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'b' )
    hold on
    plot(xx,speval(solution.Xp,1,solution.TSeg_Bar,xx),'r' )
    plot(xx,speval(solution.Xp,2,solution.TSeg_Bar,xx),'b.' )
    xlabel('Time [s]')
    ylabel('Velocity [m/s]')
    grid on
    
    figure
    plot([solution.T(:,1); solution.tf],speval(solution.Up,1,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'b' )
    hold on
    plot([solution.T(1,1); solution.tf],[problem.inputs.ul, problem.inputs.ul],'r' )
    plot([solution.T(1,1); solution.tf],[problem.inputs.uu, problem.inputs.uu],'r' )
    plot(xx,speval(solution.Up,1,solution.TSeg_Bar,xx),'b' )
    xlabel('Time [s]')
    grid on
    ylabel('Control Input')
    legend('u [N]')
else
    xx=linspace(solution.T(1,1),solution.T(end,1),1000);
    figure
    subplot(2,1,1)
    plot(solution.T(:,1),speval(solution.Xp,1,solution.T(:,1)),'r')
    ylim([0 max(speval(solution.Xp,1,solution.T(:,1)))])
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('Position [m]')
    grid on
    subplot(2,1,2)
    plot(solution.T(:,1),speval(solution.Xp,2,solution.T(:,1)),'b')
    xlabel('Time [s]')
    ylabel('Velocity [m/s]')
    grid on

    figure
    plot(solution.T(:,1),speval(solution.Up,1,solution.T),'-' )
    hold on
    plot([solution.T(1,1); solution.tf],[problem.inputs.ul, problem.inputs.ul],'r' )
    plot([solution.T(1,1); solution.tf],[problem.inputs.uu, problem.inputs.uu],'r' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    grid on
    ylabel('Control Input')
    legend('u [N]')
end