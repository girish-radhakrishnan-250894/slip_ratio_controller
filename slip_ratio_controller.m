function [z_dot, O_controller] = slip_ratio_controller(t, z, input)
%slip_ratio_controller Wrapper function that contains the slip control
%algorithm
%   This function contains the code of the slip control algorithm. It is a
%   wrapper function that calls the equations of motion. It calculates the
%   control inputs and then passes these control inputs to the equation
%   model

% INPUTS
%   t               Current time-step of the numerical integration process
%   z               Augmented state vector. This vector contains the state
%                   variables of the flexible drive shaft, slip error and low pass
%                   filter
%   input           Input struct that contains the model parameters

% OUTPUTS
%   z_dot           Augment state vector derivative
%   O_controller    Any outputs that are required 

%% Initializing state variables (necessary for control action)

% Integral of the slip error 
integral_e_slip = z(5);

% State variables of the low-pass filter which is used to filter the
% feedforward control action 
z_lpf = z(6:end);

%% Command input

% Interpolating the command torque requested by the user
M_command = interp1(input.time, input.motor_torque, t, 'linear');
%% Slip Ratio

% Calling the vehicle model once to calculate the slip ratio of the current
% time-step and angular acceleration of the wheel
[~, O_model_temp] = equation_of_motion(z,0,input);

kappa = O_model_temp(1);

omega_dot_wheel = O_model_temp(4);
%% Target slip ratio

% We only want a saturating control action when the slip ratio is more than
% 10%. The code below sets the target slip accordingly 

kappa_target = kappa;

% A target slip is set only when then the measured slip is more than 10%
% (in braking or acceleration)
% The controller will not have any error (and hence no control action)
% until this limit of 10% is breached

if abs(kappa) > 0.1 % Target of 10% slip
    kappa_target = sign(kappa)*0.1;
end

%% Slip Ratio Error

% Slip Error
e_slip = kappa_target - kappa;

% Time-derivative of slip error
e_dot_slip = -omega_dot_wheel*input.Rw/input.Vx;

% The goal of the controller is to limit the driving torque to ensure slip
% is below 10%. The goal is NOT to continuously track a given slip ratio
% target. Since there is not continuous tracking of a reference, the built
% up integral of the error has no way of going to 0. Therefore, it must be
% reset
% If the error is below a threshold, then the integral error is reset
if abs(e_slip) < 0.0005
    % Reseting the integral error because we don't want the integral action
    % to kick in when the slip is below 10%
    integral_e_slip = 0;
end

%% Control Action

% Feedward control action obtained from the low-pass filter
F_operating_point_filtered = input.C_lpf*z_lpf;

% PID Control Design
kp = 150000;
ki = 1000;
kd = 2000;

% Slip Controller Design
F_slip =  -F_operating_point_filtered + kp*e_slip + ki*integral_e_slip + kd*e_dot_slip;

M_slip = input.switch_abs*F_slip*input.Rw;

%% Saturated Motor Torque

motor_torque = M_command + M_slip;

%% System Dynamics

% Calling the drive-shaft model  with the control inputs
[q_dot, O_model] = equation_of_motion(z,motor_torque,input);

%% Low-Pass Filter Dynamics

% Calculating the feedforward force (tire operating point)
F_operating_point = ((motor_torque - omega_dot_wheel*input.Im)/input.Rw);

% Passing the calculated force as input to a low pass filter
u_lpf = F_operating_point;
z_dot_lpf = input.A_lpf*z_lpf + input.B_lpf*u_lpf;

%% Augmented system dynamics

z_dot = [q_dot;     % Drive-shaft dynamics
         e_slip;    % Integrating the calculated slip error
         z_dot_lpf  % Low-pass filter
         ];

O_controller = [O_model(1);     % Slip ratio
                O_model(2);     % Motor angular velocity
                O_model(3);     % Wheel angular velocity
                e_slip;         % Slip error
                M_slip;         % Control torque
                kappa_target;   % Target slip
                M_command;      % Command torque
                ];

end