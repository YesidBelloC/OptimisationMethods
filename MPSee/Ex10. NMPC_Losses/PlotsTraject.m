%% Plots States
close all
clc

Tiempo    = States.Time;
Posicion  = States.Data(:,1);
Velocidad = States.Data(:,2);
Energia   = States.Data(:,3);
Eff_mean  = mean(Eff.Data(:,1))
PotTot    = PotTot.Data(:,1);
MotLosses = MotorLosses.Data(:,1);
DrvLosses = DriverLosses.Data(:,1);
BatLosses = BatteryLosses.Data(:,1);

figure
subplot(2,2,1)
plot(Tiempo,Posicion)
grid on
xlabel('Time [s]')
ylabel('Distance [m]')
subplot(2,2,2)
plot(Tiempo,Velocidad)
grid on
xlabel('Time [s]')
ylabel('Speed [m/s]')
subplot(2,2,3)
plot(Tiempo,Energia)
grid on
xlabel('Time [s]')
ylabel('Energy [Ws]')

Dist_obj= 10000; %10000 all; 1000 NREL por problemas de estabilizacion.
object = find(abs(Posicion-Dist_obj)==min(abs(Posicion-Dist_obj)),1);

Time_traj = Tiempo(object)/60
MaxSpeed_traj = max(Velocidad(1:object))
AvSpeed_traj = mean(Velocidad(1:object))
Energy_traj = Energia(object)
Auton_Expec = (Dist_obj*72*45*3600)/(Energy_traj*1000)

PotTotMed    = mean(PotTot);
MotLossesMed = mean(MotLosses);
DrvLossesMed = mean(DrvLosses);
BatLossesMed = mean(BatLosses); 
MotLossesPor = (MotLossesMed*100)/PotTotMed
DrvLossesPor = (DrvLossesMed*100)/PotTotMed
BatLossesPor = (BatLossesMed*100)/PotTotMed
