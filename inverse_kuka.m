function q = inverse_kuka(H,myrobot)
    % q is a 6x1 vector
    a_1 = myrobot.a(1);
    a_2 = myrobot.a(2);
    a_3 = myrobot.a(3);
    % a_4 = myrobot.a(4);
    % a_5 = myrobot.a(5);
    a_6 = myrobot.a(6);

    d_1 = myrobot.d(1);
    % d_2 = myrobot.d(2);
    % d_3 = myrobot.d(3);
    d_4 = myrobot.d(4);
    % d_5 = myrobot.d(5);
    d_6 = myrobot.d(6);

    % H is 4x4 and contains R_d and o_d
    R_d = H(1:3, 1:3);
    o_d = H(1:3, 4);

    % Compute o_c

    o_c = o_d - R_d * [a_6; 0; d_6]; %note that the negative a_6 is already input into the DH parameters
    x_c = o_c(1);
    y_c = o_c(2);
    z_c = o_c(3);

    % Compute each joint value
    theta_1 = atan2(y_c, x_c);
    
    gamma_2 = atan2(a_3, d_4);
    R = real(sqrt(x_c^2 + y_c^2));
    D = ((R-a_1)^2 + (z_c-d_1)^2 - a_2^2 - d_4^2 -a_3^2)/(2*a_2*real(sqrt(d_4^2 + a_3^2)));
    theta_3 = atan2(D, real(sqrt(1-D^2))) - gamma_2;

    gamma_3 = atan2(real(sqrt(d_4^2 + a_3^2))*sin(theta_3 - pi/2 + gamma_2), a_2 + real(sqrt(d_4^2 + a_3^2))*cos(theta_3-pi/2+gamma_2));
    theta_2 = atan2(z_c-d_1, R-a_1) - gamma_3;

    % Generate R_3^0 using DH parameters in H matrices
    H_3_0 = eye(4);
    thetas=[theta_1, theta_2, theta_3];
    for i = 1:3 %iterate over the first 3 frames
        a_i = myrobot.a(i);
        alpha_i = myrobot.alpha(i);
        d_i = myrobot.d(i);
        theta_i = thetas(i);
        H_i_im1 = [ cos(theta_i)    -sin(theta_i)*cos(alpha_i)   sin(theta_i)*sin(alpha_i)   a_i*cos(theta_i);
                    sin(theta_i)    cos(theta_i)*cos(alpha_i)   -cos(theta_i)*sin(alpha_i)   a_i*sin(theta_i);
                    0               sin(alpha_i)                cos(alpha_i)                d_i;
                    0               0                           0                           1               ];
        H_3_0 = H_3_0*H_i_im1;
    end
    R_3_0 = H_3_0(1:3, 1:3);
    R_6_3 = R_3_0.' * R_d;
    % for sin(theta_5) > 0: (choose elbow up solution)
    theta_4 = atan2(R_6_3(2,3),R_6_3(1,3));
    theta_5 = atan2(real(sqrt(1-R_6_3(3,3)^2)), R_6_3(3,3));
    theta_6 = atan2(R_6_3(3,2), -R_6_3(3,1));

    q = [theta_1; theta_2; theta_3; theta_4; theta_5; theta_6];

end