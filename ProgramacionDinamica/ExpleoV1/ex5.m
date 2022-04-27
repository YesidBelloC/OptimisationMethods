clear all; close all; clc

tic
s = 'http://byu.apmonitor.com';
a = 'ex5_speed';

addpath('apm')
apm(s,a,'clear all');
apm_load(s,a,'ex5Speed.apm');
csv_load(s,a,'ex5.csv');

apm_option(s,a,'nlc.nodes',10);
apm_option(s,a,'nlc.solver',1);
apm_option(s,a,'nlc.imode',6);
apm_option(s,a,'nlc.mv_type',1);

apm_info(s,a,'MV','u1');
apm_option(s,a,'u1.status',1);
apm_option(s,a,'u1.dcost',0);

apm_info(s,a,'MV','u2');
apm_option(s,a,'u2.status',1);
apm_option(s,a,'u2.dcost',0);

output = apm(s,a,'solve');
disp(output)
y = apm_sol(s,a); z = y.x;

toc

%Analisis Para cerrar el lazo
T=z.time(3)-z.time(2)
U1=z.u1(2)
U2=z.u2(2)

%%
figure(1)

subplot(4,1,1)
plot(z.time,z.u1,'r-','LineWidth',2)
hold on
plot(z.time,z.u2,'b-','LineWidth',2)
legend('u_1 [Ft]','u_2 [Fb]')
ylabel('Manipulated')
grid on

subplot(4,1,2)
plot(z.time,z.x1,'b--','LineWidth',2)
ylabel('X1')
xlabel('Time')
grid on

subplot(4,1,3)
plot(z.time,z.x2,'g:','LineWidth',2)
ylabel('X2')
xlabel('Time')
grid on

subplot(4,1,4)
plot(z.time,z.x3,'b--','LineWidth',2)
legend('x3 [j]')
ylabel('s')
xlabel('Time')
grid on
z.x3(end)

figure(2)

subplot(2,1,1)
plot(z.time,z.fr,'r-','LineWidth',2)
hold on
plot(z.time,z.fa,'b-','LineWidth',2)
hold on
plot(z.time,z.fw,'g-','LineWidth',2)
legend('[Fr]','[Fa]','[Fw]')
ylabel('nM')
grid on

subplot(2,1,2)
plot(z.time,z.ang,'g:','LineWidth',2)
legend('ang [°]')
ylabel('Grades')
xlabel('Temps')
grid on
