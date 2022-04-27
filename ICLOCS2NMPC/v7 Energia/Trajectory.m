function [S,D]=Trajectory()

    %%
    load('ProfilDistVelAlturTiem.mat')

    figure
    subplot(3,1,1)
    plot(V_SPEEDetDISTANCEetHAUTEUR(:,1))
    title('Speed Profile')
    xlabel('Time [s]')
    ylabel('Speed [kmh]')
    grid on
    subplot(3,1,2)
    plot(V_SPEEDetDISTANCEetHAUTEUR(:,2))
    title('Distance')
    xlabel('Time [s]')
    ylabel('Distance [m]')
    grid on
    subplot(3,1,3)
    plot(V_SPEEDetDISTANCEetHAUTEUR(:,3))
    title('Height')
    xlabel('Time [s]')
    ylabel('[m]')
    grid on

    Goal = max(V_SPEEDetDISTANCEetHAUTEUR(:,2));
    %Goal = 5000;

    %% Nota: Los cambios de altura son drasticos.. hay que suavizarlos.
    Spd         =V_SPEEDetDISTANCEetHAUTEUR(:,1);
    Alt         =V_SPEEDetDISTANCEetHAUTEUR(1,3);
    count       =0;
    AlturaSuav_V=[Alt];
    for i=1:1:size(Spd,1)-1
        if Spd(i)~=0
            if Spd(i+1)~=0 & Alt==V_SPEEDetDISTANCEetHAUTEUR(i+1,3)
                count=count+1;
            else 
                AlturaSuav_V=[AlturaSuav_V linspace(Alt, V_SPEEDetDISTANCEetHAUTEUR(i+1,3), count)];
                Alt=V_SPEEDetDISTANCEetHAUTEUR(i+1,3);
                count=1;
            end
        else
            AlturaSuav_V=[AlturaSuav_V Alt];
        end
    end

    %     plot(Spd(:))
    %     hold on
    %     plot(V_SPEEDetDISTANCEetHAUTEUR(:,3)/10)
    %     hold on
    %     plot(AlturaSuav_V)

    %% Agrupo los vactores requeridos para calcular el torq
    %plot([AlturaSuav_V/max(AlturaSuav_V)-0.5;H']')
    D=V_SPEEDetDISTANCEetHAUTEUR(:,2); %Distancia
    Ang=[];
    Pend=[];
    Pend_temp=0;

    for i=1:size(D,1)-1
        if (D(i+1)-D(i))>0
            Pend_temp=(AlturaSuav_V(i+1)-AlturaSuav_V(i))/(D(i+1)-D(i));
        end
        Pend=[Pend Pend_temp];

        if abs(real(asin(Pend_temp)))>0.1 %Limitacion de pendiente= (M*a+sum(F))*R=Tmax ... a=0
            Ang=[Ang 0.1];                % Ojo... la masa es bastante porq tengo 2 pasajeros<
        else
            Ang=[Ang real(asin(Pend_temp))];
        end
    end
    S=Ang;

    % plot(AlturaSuav_V)
    % hold on
    % plot(rad2deg(Ang)*10)

end