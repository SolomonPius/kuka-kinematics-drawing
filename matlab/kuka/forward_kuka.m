function H = forward_kuka(q, myrobot)
    % q is a 6x1 vector
    % H is 4x4 vector composed of incremental transformations
    H = eye(4);

    for i = 1:6 %the q is 6x1
        a_i = myrobot.a(i);
        alpha_i = myrobot.alpha(i);
        d_i = myrobot.d(i);
        theta_i = q(i);
        H_i_im1 = [ cos(theta_i)    -sin(theta_i)*cos(alpha_i)   sin(theta_i)*sin(alpha_i)   a_i*cos(theta_i);
                    sin(theta_i)    cos(theta_i)*cos(alpha_i)   -cos(theta_i)*sin(alpha_i)   a_i*sin(theta_i);
                    0               sin(alpha_i)                cos(alpha_i)                d_i;
                    0               0                           0                           1               ];
        H = H*H_i_im1;
    end
end