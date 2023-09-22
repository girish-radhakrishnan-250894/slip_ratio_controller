
clc;
clear;
addpath(genpath(pwd));
%% Inputs

input.Im = 0.5;
input.Iw = 1.5;
input.k = 10000;
input.Cx = 10000;
input.Rw = 0.3;
input.Ng = 10;
input.Vx = 20;

load('low_pass_filter_IT1.mat')
[cA_lpf,cB_lpf,cC_lpf,cD_lpf] = tf2ss(cell2mat(shapeit_data.C_tf.Numerator),cell2mat(shapeit_data.C_tf.Denominator));
input.A_lpf = cA_lpf; input.B_lpf = cB_lpf; input.C_lpf = cC_lpf; input.D_lpf = cD_lpf;
%% Simulation Inputs

input.time =           [0 4 6 8 12 13 15 25];
input.motor_torque  = 10*[0 0 100 100 100 0 0 0];

input.switch_abs = 1;

%% Initial Conditions
% Integration options
opts = odeset("RelTol",1e-6,'MaxStep',0.005);

% Creating the augmentat state-vector
omega_wheel_0 = input.Vx/input.Rw;
omega_motor_0 = omega_wheel_0*input.Ng;
Z0 = [0 0 omega_motor_0 omega_wheel_0 0 zeros(1, size(cA_lpf, 1))];



[t,Z] = ode15s(@(t,X)slip_ratio_controller(t,X,input), [0 input.time(end)], Z0, opts);

% Output matrix initialization
n_outputs = 7;
O = zeros(length(t),n_outputs);
for i=1:length(Z)
    [~, O(i,:)] = slip_ratio_controller(t(i),Z(i,:)',input);
end


%% Plots


figure
plot(t,rad2deg(O(:,2)),'k--','LineWidth',1.5);
ylabel("Angular Velocity [deg/s]")
yyaxis right
plot(t,rad2deg(O(:,3)));

xlabel("Time [s]")
legend("\omega_{motor}", "\omega_{wheel}",Location="best")


figure
plot(t,O(:,1)*100,'LineWidth',1);
ylabel("Slip Ratio [%]")
hold on
plot(t,O(:,6)*100,'k--','LineWidth',2)
xlabel("Time [s]")
legend("kappa","kappa_{target}",Location="best")

figure
plot(t,O(:,4)*100,'LineWidth',1.5);
xlabel("Time [s]")
ylabel("Slip Error [%]")

figure
plot(t,O(:,7),'k--','LineWidth',1)
hold on
plot(t,O(:,5),'LineWidth',1)
ylabel("Control Torque [Nm]")
xlabel("Time [s]")
legend("M_{command}", "M_{controller}")
