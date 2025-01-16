function T = dh_proximal(theta, d, a, alpha)
    cos_alpha = round(cos(alpha), 1);
    sin_alpha = round(sin(alpha), 1);

    %D-H transformation matrix
    T = [ cos(theta),            -sin(theta),              0,             a;
          sin(theta)*cos_alpha, cos(theta)*cos_alpha,  -sin_alpha,   -sin_alpha*d;
          sin(theta)*sin_alpha,  cos(theta)*sin_alpha,   cos_alpha,    cos_alpha*d;
          0,                      0,                      0,                       1];

end