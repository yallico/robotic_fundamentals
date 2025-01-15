function plot_oriented_cylinder(radius, height, origin, orientation)
    % create a unit cylinder along the Z-axis
    [X, Y, Z] = cylinder(radius);
    Z = Z * height;
    R = eul2rotm(orientation); % assuming orientation is in Euler angles

    %X, Y, Z matrices into lists of 3D points
    points = [X(:), Y(:), Z(:)]';

    %Rotate all points in a single matrix operation
    rotated_points = R * points;

    X_rot = reshape(rotated_points(1, :) + origin(1), size(X));
    Y_rot = reshape(rotated_points(2, :) + origin(2), size(Y));
    Z_rot = reshape(rotated_points(3, :) + origin(3), size(Z));

    surf(X_rot, Y_rot, Z_rot, 'EdgeColor', 'none', 'FaceColor', [0.6, 1.0, 0.6]);

    % edge top and bottom circular end caps with black edge
    % bottom cap
    patch(X_rot(1, :), Y_rot(1, :), Z_rot(1, :), [0.6, 1.0, 0.6], 'EdgeColor', 'k', 'LineWidth', 0.5);
    % top cap
    patch(X_rot(2, :), Y_rot(2, :), Z_rot(2, :), [0.6, 1.0, 0.6], 'EdgeColor', 'k', 'LineWidth', 0.5);

    hold on;

end
