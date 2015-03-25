function [AC] = miniMUTT_config()
% function [AC] = miniMUTT_config()
%
%
% mini MUTT configuration file. Sets aircraft parameters.
% Called from: UAV_config.m
%
% University of Minnesota 
% Aerospace Engineering and Mechanics 
% Copyright 2011 Regents of the University of Minnesota. 
% All rights reserved.
%
% SVN Info: $Id: miniMUTT_config.m 765 2012-01-25 20:14:05Z murch $


% XXX allow to switch between different structural models (DLR and UMNs)
% using and input variable

% XXX add propeller model and accelerometers in the wings 

% XXX update actuator model

% XXX add geometric data even though it is not needed at the moment?

AC.aircraft = 'miniMUTT';

% load mini MUTT model data
load miniMUTT_Dynamics

%% Inertia, Mass and CG location
% CG location not required for the mini MUTT-Sim at the moment

% Gross aircraft mass [kg] (including relevant hardware)
AC.Mass  = miniMUTT_dyn.modes.m_AC;

% Gross moments of inertia [Jx Jy Jz Jxz] [kg*m^2]
AC.Inertia.Matrix = miniMUTT_dyn.modes.I_B;


%% Aircraft Geometric Parameters

% Mean aerodynamic chord [m]
AC.Geometry.c = miniMUTT_dyn.geom.c_ref;

% no other geometric parameters required for the mini MUTT-Sim as the
% aerodynamic model directly returns forces and moments

%% Aero coefficients

% Rational Function Approximation (RFA) of the unsteady aerodynamics
% obtained by DLM code
AC.Aero.RFA = miniMUTT_dyn.aero.RFA_Qhhx_nln;

%DT1 filter is used to numerically obtain the accelerations required by the
%aeromodel. 
AC.Aero.DT1_omega = 300; %bandwidth of the DT1 filter



%% Structural Dynamics
% FEM model of the structural dynamics given by mass, damping and stiffness
% matrices

AC.Struct.MassMatrix = miniMUTT_dyn.modes.Mff;
AC.Struct.InvMassMatrix = inv(miniMUTT_dyn.modes.Mff);
AC.Struct.Damping = miniMUTT_dyn.modes.Bff;
AC.Struct.Stiffness = miniMUTT_dyn.modes.Kff;

%% PROPULSION

% no propulsion model available!

%% Configure Actuators and Initial Conditions
% No mini-MUTT servos data available yet
% Bandwidth data taken from BFF servos
% Rates limits not accurate
d2r = pi/180;                         % Degrees to Radians conversion

% Flutter suppression L1
AC.Actuator.L1.BW = 70;            % [Hz]
AC.Actuator.L1.RateLim = 1000*d2r; % [rad/s]
AC.Actuator.L1.PosLim = 25*d2r;   % [rad]
AC.Actuator.L1.NegLim = -25*d2r;  %[rad]

% Left Aileron L2
AC.Actuator.L2.BW = 30;            % [Hz]
AC.Actuator.L2.RateLim = 500*d2r; % [rad/s]
AC.Actuator.L2.PosLim = 25*d2r;   % [rad]
AC.Actuator.L2.NegLim = -25*d2r;  %[rad]

% Elevator L3
AC.Actuator.L3.BW = 30;            % [Hz]
AC.Actuator.L3.RateLim = 500*d2r; % [rad/s]
AC.Actuator.L3.PosLim = 25*d2r;   % [rad]
AC.Actuator.L3.NegLim = -25*d2r;  %[rad]

% Flutter suppression L4
AC.Actuator.L4.BW = 70;            % [Hz]
AC.Actuator.L4.RateLim = 1000*d2r; % [rad/s]
AC.Actuator.L4.PosLim = 25*d2r;   % [rad]
AC.Actuator.L4.NegLim = -25*d2r;  %[rad]

% Flutter suppression R1
AC.Actuator.R1.BW = 70;            % [Hz]
AC.Actuator.R1.RateLim = 1000*d2r; % [rad/s]
AC.Actuator.R1.PosLim = 25*d2r;   % [rad]
AC.Actuator.R1.NegLim = -25*d2r;  %[rad]

% Right Aileron R2
AC.Actuator.R2.BW = 30;            % [Hz]
AC.Actuator.R2.RateLim = 500*d2r; % [rad/s]
AC.Actuator.R2.PosLim = 25*d2r;   % [rad]
AC.Actuator.R2.NegLim = -25*d2r;  %[rad]

% Elevator R3
AC.Actuator.R3.BW = 30;            % [Hz]
AC.Actuator.R3.RateLim = 500*d2r; % [rad/s]
AC.Actuator.R3.PosLim = 25*d2r;   % [rad]
AC.Actuator.R3.NegLim = -25*d2r;  %[rad]

% Flutter suppression R4
AC.Actuator.R4.BW = 70;            % [Hz]
AC.Actuator.R4.RateLim = 1000*d2r; % [rad/s]
AC.Actuator.R4.PosLim = 25*d2r;   % [rad]
AC.Actuator.R4.NegLim = -25*d2r;  %[rad]

%Throttle
AC.Actuator.throttle.PosLim = 1;  % [nd]
AC.Actuator.throttle.NegLim = 0; %[nd]


%% Configure Sensor Noise Parameters
% same as UltraSticks
% XXX accelerometers in the wings are missing
AC.Sensors.NoiseOn = 1;
AC.Sensors.IMU.p_noise = 0.000001; % rad/s
AC.Sensors.IMU.q_noise = 0.000001; % rad/s
AC.Sensors.IMU.r_noise = 0.000001; % rad/s
AC.Sensors.IMU.hx_noise =150;   % nT 
AC.Sensors.IMU.hy_noise =150;   % nT
AC.Sensors.IMU.hz_noise =80000; % nT
AC.Sensors.IMU.ax_noise =0.0008; % m/s^2
AC.Sensors.IMU.ay_noise =0.004;  % m/s^2
AC.Sensors.IMU.az_noise =0.004;  % m/s^2
AC.Sensors.AirData.ias_noise = 0.001; % m/s
AC.Sensors.AirData.h_noise = 0.02; % m
% Alpha/beta noise levels need to be verified with flight data
AC.Sensors.AirData.alpha_noise = 0.0000002; % rad
AC.Sensors.AirData.beta_noise = 0.0000002; % rad
AC.Sensors.AirData.Pd_noise = 0.00000015; % Kpa, AMS 5812
AC.Sensors.AirData.Ps_noise = 0.0000008; % Kpa, AMS 5812


%% Configure Sensor Bias Parameters
AC.Sensors.IMU.p_bias = 0; % rad/s
AC.Sensors.IMU.q_bias = 0; % rad/s
AC.Sensors.IMU.r_bias = 0; % rad/s
AC.Sensors.IMU.hx_bias = 0;   % nT 
AC.Sensors.IMU.hy_bias = 0;   % nT
AC.Sensors.IMU.hz_bias = 0; % nT
AC.Sensors.IMU.ax_bias = 0; % m/s^2
AC.Sensors.IMU.ay_bias = 0;  % m/s^2
AC.Sensors.IMU.az_bias = 0;  % m/s^2
AC.Sensors.AirData.ias_bias = 0; % m/s
AC.Sensors.AirData.h_bias = 0; % m
AC.Sensors.AirData.alpha_bias = 0; % rad
AC.Sensors.AirData.beta_bias = 0; % rad
AC.Sensors.AirData.Pd_bias = 0; % Kpa, AMS 5812
AC.Sensors.AirData.Ps_bias = 0; % Kpa, AMS 5812

%% Configure Sensor Scale Factor Parameters
AC.Sensors.IMU.p_scf = 1; % rad/s
AC.Sensors.IMU.q_scf = 1; % rad/s
AC.Sensors.IMU.r_scf = 1; % rad/s
AC.Sensors.IMU.hx_scf = 1;   % nT 
AC.Sensors.IMU.hy_scf = 1;   % nT
AC.Sensors.IMU.hz_scf = 1; % nT
AC.Sensors.IMU.ax_scf = 1; % m/s^2
AC.Sensors.IMU.ay_scf = 1;  % m/s^2
AC.Sensors.IMU.az_scf = 1;  % m/s^2
AC.Sensors.AirData.ias_scf = 1; % m/s
AC.Sensors.AirData.h_scf = 1; % m
AC.Sensors.AirData.alpha_scf = 1; % rad
AC.Sensors.AirData.beta_scf = 1; % rad
AC.Sensors.AirData.Pd_scf = 1; % Kpa, AMS 5812
AC.Sensors.AirData.Ps_scf = 1; % Kpa, AMS 5812
