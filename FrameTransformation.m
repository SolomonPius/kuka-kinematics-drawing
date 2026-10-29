function pBaseFrame = FrameTransformation(pWorkspace)
    % FRAMETRANSFORMATION  Convert a point from the paper (workspace) frame to the robot base frame.
    % This function returns the coordinate representation of a point in the robot baseframe
    % given the coordinates of the point in the workspace frame.
    
    %-------------------------- Calibration ----------------------------%
    % Xi are the three measured calibration points on the paper (3x1, mm).
    % If the robot is recalibrated, update deltajoint.m as well.
    
    X1 = [675.71; 179.56; 28.02];
    X2 = [681.04; -148.32; 24.14];
    X3 = [620.14; -319.63; 25.31];
	

    % Finding plane coordinates: a*x + b*y + c*z = 1
    M = [X1'; X2'; X3'];
    params = pinv(M)*[1 1 1]';
    a = params(1);
    b = params(2);
    c = params(3);
    v = [a b c]';  % plane normal vector; should be close to [0;0; 1] !
    
    % Rotation matrix; mapping from Base-frame to workspace-frame: R*v1 = v2
    v2 = v/norm(v);
    v1 = [0 0 1]';    
    rotV = vrrotvec(v1,v2);    
    R = vrrotvec2mat(rotV);
    
    % Calculating transformation matrix:  H*pWorkspace = pBaseFrame
    xOrig = 0;
    yOrig = 0;
    zOrig = (1 -a*xOrig -b*yOrig)/c;    
    Orig = [xOrig  yOrig  zOrig]';    
    H = [R, Orig; zeros(1,3), 1];    
    
    tmp = H*[pWorkspace;1];
    pBaseFrame = tmp(1:3);   % any point on the table is converted back
                             % to robot base-frame. 
                             % "pBaseFrame" can be used by robot
                             % inverse kinematics to find proper joint
                             % angles!
end
    
    
    
    
    
    


