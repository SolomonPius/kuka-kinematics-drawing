
function H = forward(joint,myrobot)
	theta1 = joint(:,1);
	theta2 = joint(:,2);
	theta3 = joint(:,3);
	theta4 = joint(:,4);
	theta5 = joint(:,5);
	theta6 = joint(:,6);
	
	d1 = myrobot.d(1);
	a2 = myrobot.a(2);
	d2 = myrobot.d(2);
	d4 = myrobot.d(4);
	d6 = myrobot.d(6);
	
	H01 = [ cos(theta1) 0 sin(theta1) 0;
	        sin(theta1) 0 -cos(theta1) 0;
	        0 1 0 d1;
	        0 0 0 1];
	
	H12 = [ cos(theta2) -sin(theta2) 0 a2*cos(theta2);
	        sin(theta2) cos(theta2) 0 a2*sin(theta2);
	        0 0 1 d2;
	        0 0 0 1];
	
	H23 = [ cos(theta3) 0 sin(theta3) 0;
	        sin(theta3) 0 -cos(theta3) 0 ;
	        0 1 0 0;
	        0 0 0 1];
	
	H34 = [ cos(theta4) 0 -sin(theta4) 0;
	        sin(theta4) 0 cos(theta4) 0;
	        0 -1 0 d4;
	        0 0 0 1];

	H45 = [cos(theta5),0, sin(theta5),  0;
	       sin(theta5),0, -cos(theta5),  0;
	       0, 1, 0, 0;
	       0, 0, 0, 1];
	
	H56 = [cos(theta6), -sin(theta6), 0, 0;
	       sin(theta6), cos(theta6), 0, 0;
	       0, 0, 1, d6;
	       0, 0, 0, 1];
	
	H = H01*H12*H23*H34*H45*H56;
	
end

