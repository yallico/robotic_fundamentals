% Import Robotics Toolbox
% You need to install the toolbox before running this command
% addpath('path_to_robotics_toolbox');
clc
clear
close all

% Define symbolic variables for joint angles (for inverse kinematics or symbolic calculations)
syms theta1 theta2 theta3 theta4 theta5

% Define the link lengths and offsets
d1 = 1;   % Base offset (along the z-axis)
L1_len = 1; % Link 1 length
L2_len = 1; % Link 2 length
L3_len = 1; % Link 3 length (for offset along z-axis)

% Define the Denavit-Hartenberg parameters for each link
L1 = Link('revolute', 'd', d1, 'a', 0, 'alpha', pi/2, 'qlim', [-pi, pi]);  % Revolute joint 1, rotates around z-axis
L2 = Link('revolute', 'd', 0, 'a', L1_len, 'alpha', 0, 'qlim', [-pi, pi]);  % Revolute joint 2, rotates around x-axis
L3 = Link('revolute', 'd', 0, 'a', L2_len, 'alpha', 0, 'qlim', [-pi, pi]);  % Revolute joint 3, rotates around x-axis
L4 = Link('revolute', 'd', 0, 'a', 0, 'alpha', -pi/2, 'qlim', [-pi, pi]);    % Revolute joint 4, twist about z-axis
L5 = Link('revolute', 'd', L3_len, 'a', 0, 'alpha', 0, 'qlim', [-pi, pi]);  % Revolute joint 5, rotates along z-axis

%+ve or -ve alpha imapct the initial position of the end effector

% Combine links to form the robot (5-DOF Revolute robot)
RRRRR_robot = SerialLink([L1 L2 L3 L4, L5], 'name', 'RRRRR Arm');

% Set the initial joint variables (angles)
q = [0, 0, 0, 0, 0];  % Example joint angles

% Plot and visualize the robot at the given joint configuration
figure;
RRRRR_robot.plot(q);  % Visualize the robot
RRRRR_robot.teach();  % Interactive tool to move joints