% PUMA560_DEMO  Forward and inverse kinematics of the PUMA 560 in simulation.
%   Requires Peter Corke's Robotics Toolbox for MATLAB.

clear;
close;

DH = [0, 76,     0,     pi/2;
      0, -23.65, 43.23, 0;
      0, 0,      0,     pi/2;
      0, 43.18,  0,     -pi/2;
      0, 0,      0,     pi/2;
      0, 20,     0,     0];
myrobot = mypuma560(DH)

% Animate a sample joint trajectory
q = zeros(200,6);
q(:,1)=linspace(0,pi,200);
q(:,2)=linspace(0,pi/2,200);
q(:,3)=linspace(0,pi,200);
q(:,4)=linspace(pi/4,3*pi/4,200);
q(:,5)=linspace(-pi/3,pi/3,200);
q(:,6)=linspace(0,2*pi,200);

figure()
view(3)
plot(myrobot,q)

%%

% Forward kinematics: trace the end effector path

o = zeros(200,3);

for i = 1:200
    temp_H = forward(q(i,:), myrobot);
    o(i, :)= temp_H(1:3,4);
end


plot3(o(:,1),o(:,2),o(:,3),'r');
hold on;
plot(myrobot,q);

%% 

% Inverse kinematics: follow a straight-line path
H = [cos(pi/4) -sin(pi/4) 0 20; sin(pi/4) cos(pi/4) 0 23; 0 0 1 15; 0 0 0 1];
output = inverse(H, myrobot);


t = linspace(0, 1, 100);
t_1 = linspace(1, 0, 100);
o_x = 30*t + 10*t_1;
o_y = 30*t + 23*t_1;
o_z = 100*t + 15*t_1;
d = [o_x; o_y; o_z].';
R = eul2tr(0,0,pi/4);

q = zeros(100,6);
for i = 1:100
    
    cur_H = R;
    cur_H(1:3,4) = d(i,:).';
    % disp(cur_H);
    q(i,:) = inverse(cur_H,myrobot).';
end

plot3(d(:,1),d(:,2),d(:,3), "r");
hold on;
plot(myrobot,q);
