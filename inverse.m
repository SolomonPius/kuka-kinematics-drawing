function q = inverse(H,myrobot)
	d_1 = myrobot.d(1);
	a_2 = myrobot.a(2);
	d_2 = myrobot.d(2);
	d_4 = myrobot.d(4);
	d_6 = myrobot.d(6);
	x_c = H(1,4) - d_6*H(1,3);
	y_c = H(2,4) - d_6*H(2,3);
	z_c = H(3,4) - d_6*H(3,3) ;
	r = real(sqrt(x_c^2 + y_c^2));
	q_1 = atan2(y_c, x_c) - atan2(-d_2, real(sqrt(r^2 - (-d_2)^2)));
	D = (r^2 - d_2^2 + (z_c - d_1)^2 - a_2^2 - d_4^2)/(2*a_2*d_4);
	q_3 = atan2(D, real(sqrt(1-D^2)));
	q_2 = atan2(z_c - d_1, real(sqrt(r^2 - d_2^2))) - atan2(d_4*sin(q_3-pi/2), a_2 + d_4*cos(q_3 - pi/2));
	
	theta1 = q_1;
	theta2 = q_2;
	theta3 = q_3;
	
	H01 = [ cos(theta1) 0 sin(theta1) 0;
	        sin(theta1) 0 -cos(theta1) 0;
	        0 1 0 d_1;
	        0 0 0 1;];
	
	H12 = [ cos(theta2) -sin(theta2) 0 a_2*cos(theta2);
	        sin(theta2) cos(theta2) 0 a_2*sin(theta2);
	        0 0 1 d_2;
	        0 0 0 1;];
	
	H23 = [ cos(theta3) 0 sin(theta3) 0;
	        sin(theta3) 0 -cos(theta3) 0 ;
	        0 1 0 0;
	        0 0 0 1;];
	% choosing the solution for sin(q_5) > 0 
	H03 = H01*H12*H23;
	H36 = H03.'*H;
	q_4 = atan2(H36(2,3),H36(1,3));
	q_5 = atan2(real(sqrt(1-H36(3,3)^2)), H36(3,3));
	q_6 = atan2(H36(3,2), -H36(3,1));
	
	q = [q_1; q_2; q_3; q_4; q_5; q_6];
end

