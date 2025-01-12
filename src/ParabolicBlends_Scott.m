% Lynxmotion Arm Parameters
l1 = 1.0; % Shoulder link length
l2 = 1.0; % Elbow link length
l3 = 0.5; % Wrist link length

% Define target end-effector positions (example: M in YZ plane)
M_points = [
    0, 0, 0.5;   % Pos_1 start bottom-left
    0, 5, 0.5;   % Pos_2 top-left
    0, 2.5, 3;   % Pos_3 middle-bottom
    0, 5, 5;     % Pos_4 top-right
    0, 0, 5      % Pos_5 bottom-right
];

% Time parameters
tb = 0.5; % Blend time (s)
tf = 2.0; % Total time per segment (s)
dt = 0.01; % Time step (s)
time_vector = 0:dt:tf;

% Loop through each segment of the 'M'
for i = 1:size(M_points, 1) - 1
    % Extract start and end positions
    start_point = M_points(i, :);
    end_point = M_points(i + 1, :);
    
    % Solve inverse kinematics for start and end points
    theta0 = calculate_joint_angles(start_point, l1, l2, l3);
    thetaf = calculate_joint_angles(end_point, l1, l2, l3);
    
    % Print summary for this segment
    fprintf('\nSegment %d: From M Point %d to %d\n', i, i, i + 1);
    fprintf('Joints       Start θ0 ( (θ1, θ2, θ3, θ4, θ5)        End θf (θ1, θ2, θ3, θ4, θ5)\n');
    
    for joint = 1:5
        % Extract joint angles for this segment
        theta_start = theta0(joint);
        theta_end = thetaf(joint);
        theta_dot = (theta_end - theta_start) / (tf - tb);
        theta_ddot = theta_dot / tb;
        
        % Generate trajectories
        theta_traj = generate_parabolic_blend_trajectory(theta_start, theta_end, theta_dot, theta_ddot, tb, tf, dt);
        
        % Extract start and end values of each phase
        blend1_start = theta_traj(1);                % Start of Blend 1
        blend1_end = theta_traj(find(time_vector == tb, 1)); % End of Blend 1
        linear_start = blend1_end;                  % Start of Linear
        linear_end = theta_traj(find(time_vector == (tf - tb), 1)); % End of Linear
        blend2_start = linear_end;                  % Start of Blend 2
        blend2_end = theta_traj(end);               % End of Blend 2
        
        % Print values for this joint
        fprintf('  Joint %d   (%6.4f, %6.4f, %6.4f)        (%6.4f, %6.4f, %6.4f)\n', ...
            joint, blend1_start, linear_start, blend2_start, blend1_end, linear_end, blend2_end);
    end
end

%% Helper Functions
function theta_traj = generate_parabolic_blend_trajectory(theta0, thetaf, theta_dot, theta_ddot, tb, tf, dt)
    t = 0:dt:tf;
    theta_traj = zeros(size(t));
    for i = 1:length(t)
        if t(i) <= tb
            % Acceleration phase
            theta_traj(i) = theta0 + 0.5 * theta_ddot * t(i)^2;
        elseif t(i) <= tf - tb
            % Constant velocity phase
            theta_traj(i) = theta0 + theta_dot * (t(i) - tb / 2);
        else
            % Deceleration phase
            theta_traj(i) = thetaf - 0.5 * theta_ddot * (tf - t(i))^2;
        end
    end
end

function theta = calculate_joint_angles(point, l1, l2, l3)
    x = point(1);
    y = point(2);
    z = point(3);
    
    % Compute θ1 (base rotation)
    theta1 = atan2(y, x);
    
    % Compute r (distance to target in the XY plane)
    r = sqrt(x^2 + y^2);
    
    % Compute θ2 and θ3 using geometric IK
    cos_theta2 = (r^2 + z^2 - l1^2 - l2^2) / (2 * l1 * l2);
    cos_theta2 = min(max(cos_theta2, -1), 1); % Clamp to valid range
    theta2 = acos(cos_theta2);
    theta3 = atan2(z, r) - theta2;
    
    % Assuming wrist alignment for simplicity
    theta4 = 0;
    theta5 = 0;
    theta = [theta1, theta2, theta3, theta4, theta5];
end
