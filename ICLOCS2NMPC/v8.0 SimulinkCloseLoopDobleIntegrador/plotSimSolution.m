
figure
plot(T,X(:,1),'b' )
hold on
xlabel('Time [s]')
ylabel('Distance [m]')
grid on

figure
plot(T,X(:,2),'b' )
hold on
xlabel('Time [s]')
ylabel('Velocity [m/s]')
grid on

figure
plot(T,U(:,1),'b')
hold on
xlabel('Time [s]')
ylabel('Control Input (Torque) [Nm]')
grid on