l1 = 1.0; 
l2 = 1.0; 
l3 = 0.5; 


M_points = [
    0, 0, 0.5;   
    0, 5, 0.5;  
    0, 2.5, 3;   
    0, 5, 5;    
    0, 0, 5     
];


tb = 0.5; 
tf = 2.0; 
dt = 0.01; 
time_vector = 0:dt:tf;


validation_file = fopen('validation_results.csv', 'w');
fprintf(validation_file, 'Point,X,Y,Z,Reachable\n');


reachable_points = [];
fprintf('\nValidation of Target Points:\n');
fprintf('Point   X      Y      Z      Reachable\n');
for i = 1:size(M_points, 1)
    point = M_points(i, :);
    try
        
        theta = calculate_joint_angles(point, l1, l2, l3);
       
        if all(theta >= -pi & theta <= pi)
            reachable_points = [reachable_points; point];
            reachable = 'Yes';
        else
            reachable = 'No';
        end
    catch
        reachable = 'No';
    end
    
    %Print this shit!
    fprintf('%5d   %5.2f   %5.2f   %5.2f   %s\n', i, point(1), point(2), point(3), reachable);
    
   
    fprintf(validation_file, '%d,%6.4f,%6.4f,%6.4f,%s\n', i, point(1), point(2), point(3), reachable);
end

fclose(validation_file);


M_points = reachable_points;


trajectory_file = fopen('trajectory_results.csv', 'w');
fprintf(trajectory_file, 'Segment,Joint,Blend1_Start,Linear_Start,Blend2_Start,Blend1_End,Linear_End,Blend2_End\n');


for i = 1:size(M_points, 1) - 1
   
    start_point = M_points(i, :);
    end_point = M_points(i + 1, :);
    

    theta0 = calculate_joint_angles(start_point, l1, l2, l3);
    thetaf = calculate_joint_angles(end_point, l1, l2, l3);
    
  
    fprintf('\nSegment %d: From M Point %d to %d\n', i, i, i + 1);
    fprintf('Joint   Blend1_Start  Linear_Start  Blend2_Start  Blend1_End  Linear_End  Blend2_End\n');
    
    for joint = 1:5
       
        theta_start = theta0(joint);
        theta_end = thetaf(joint);
        theta_dot = (theta_end - theta_start) / (tf - tb);
        theta_ddot = theta_dot / tb;
        
    
        theta_traj = generate_parabolic_blend_trajectory(theta_start, theta_end, theta_dot, theta_ddot, tb, tf, dt);
        
   
        blend1_start = theta_traj(1);
        blend1_end = theta_traj(find(time_vector == tb, 1));
        linear_start = blend1_end;
        linear_end = theta_traj(find(time_vector == (tf - tb), 1));
        blend2_start = linear_end;
        blend2_end = theta_traj(end);
        
      
        fprintf(trajectory_file, '%d,%d,%6.4f,%6.4f,%6.4f,%6.4f,%6.4f,%6.4f\n', ...
            i, joint, blend1_start, linear_start, blend2_start, blend1_end, linear_end, blend2_end);
        
  
        fprintf('%5d   %12.4f   %12.4f   %12.4f   %10.4f   %10.4f   %10.4f\n', ...
            joint, blend1_start, linear_start, blend2_start, blend1_end, linear_end, blend2_end);
    end
end


fclose(trajectory_file);

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
