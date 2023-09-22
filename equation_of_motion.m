function [q_dot, O_model] = equation_of_motion(q, motor_torque, in)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here

%% Initializing state variables

theta_motor = q(1);
theta_wheel = q(2);

omega_motor = q(3);
omega_wheel = q(4);


%% Driveshaft torsion

theta_shaft = theta_motor/in.Ng - theta_wheel;

%% Tire slip ratio

kappa = (omega_wheel * in.Rw  - in.Vx)/in.Vx;

%% Tire force

F_tire = in.Cx * kappa;

%% System Dynamics

omega_dot_motor = (-in.k*theta_shaft + motor_torque)/in.Im;
omega_dot_wheel = (-F_tire*in.Rw + in.k*theta_shaft)/in.Iw;

q_dot = [omega_motor;
         omega_wheel;
         omega_dot_motor;
         omega_dot_wheel
         ];

O_model = [kappa;
           omega_motor;
           omega_wheel;
           omega_dot_wheel
           ];
end