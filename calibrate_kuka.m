% CALIBRATE_KUKA  Calibrate the KUKA tool offsets and test the drawing setup.
%   Requires the robot interface functions (startConnection, moveAxis,
%   getAngles, setAngles) provided by the robot's control environment.

clear;
close;

startConnection

%%
% 1. Calibration of DH parameters
% Jog the robot axis by axis to touch the pencil to three points on the paper.

% Moving the robot by each axis

% A sample axis is shown here. Of course, the choice of axis and velocity (direction) were modified on the fly
moveAxis(2, 0.01);

getAngles();
% the getAngles() command was used to output the angle readings to the terminal, 
% which were written down for each calibration point.
% Additionally, the (x,y,z) coordinates of the pencil w.r.t baseframe
% was recorded. Shown below.

% Measured calibration points (also used in deltajoint.m and FrameTransformation.m)
X1 = [675.71; 179.56; 28.02];
X2 = [681.04; -148.32; 24.14];
X3 = [620.14; -319.63; 25.31];
Q1 = [0.2492    0.6360    0.0428    0.0255    0.8954   -0.0161];
Q2 = [-0.2250    0.6302    0.0429    0.0255    0.8954   -0.0161];
Q3 = [   -0.4864    0.6320    0.0429    0.0255    0.8954   -0.0161];

%% 
% 2. Solve for the tool offset corrections (delta_a6, delta_d6) that
%    minimize forward kinematics error at the three calibration points
delta = fminunc(@deltajoint, [0 0]);
disp(delta)
% returns [1.2958 1.6840]

% Using delta to create new robot with the updated parameters
myrobot = mykuka_search(delta);

%%
% 3. Test the calibration at point X1
H = eye(4);
H(1:3,1:3) = [0 0 1; 0 -1 0; 1 0 0];
H(1:3,4) = X1; % X1 from above
q = inverse_kuka(H, myrobot);
setAngles(q, 0.01); % 0.04 was too fast
% the robot behaved as expected

%%
% 4. Test the paper frame -> base frame transformation
p_workspace = [600; 100; 10];
p_baseframe = FrameTransformation(p_workspace);
R = [0 0 1; 0 -1 0; 1 0 0];
H = [R p_baseframe; zeros(1,3) 1];
q = inverse_kuka(H, myrobot);
%%
setAngles(q, 0.01);
% This brought the pencil tip about 10mm above the board, as expected
%%
% 5. Drawing scripts: draw_line.m, draw_circle.m and draw_design.m
% 