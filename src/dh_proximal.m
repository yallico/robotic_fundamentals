function T = dh_proximal(theta, d, a, alpha)
    % D-H transformation matrix using the proximal method.
    % Inputs:
    %   theta (in radians)
    %   d
    %   a
    %   alpha

    % Define trigonometric functions with rounding to 1 decimal place
    cos_alpha = round(cos(alpha), 1);
    sin_alpha = round(sin(alpha), 1);

    % Compute the D-H transformation matrix
    T = [ cos(theta),            -sin(theta),              0,             a;
          sin(theta)*cos_alpha, cos(theta)*cos_alpha,  -sin_alpha,   -sin_alpha*d;
          sin(theta)*sin_alpha,  cos(theta)*sin_alpha,   cos_alpha,    cos_alpha*d;
          0,                      0,                      0,                       1];

    % Output:
    %   T: transformation matrix from frame i-1 to frame i
end