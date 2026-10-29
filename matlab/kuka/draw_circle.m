% X_workspace is a 3x100 matrix. It represents the paths to be drawn on the paper in the paper frame.
% We want to define a circle. The Z will remain constant, but X and Y will change.
x_val_workspace = 50*cos(linspace(0,2*pi,100))+620;
y_val_workspace = 50*sin(linspace(0,2*pi,100));
X_workspace =   [x_val_workspace;
                 y_val_workspace;
                 -5.*ones(1, 100)];

% X_baseframe is a 3x100 matrix with the same points as X_workspace but expressed in workspace coordinates
X_baseframe = zeros(3, 100);
for i = 1:100
    % Update each column with the FrameTransformation function
    X_baseframe(:,i) = FrameTransformation(X_workspace(:,i));
end

% Reconstruct each H matrix
% H_collection is a 4x4x100 matrix, a collection of 100 4x4 matrices
H_collection = zeros(4,4,100);
R_6_0 = [0, 0,  1;
         0, -1, 0;
         1, 0,  0];
for i = 1:100
    H_collection(1:3,1:3,i) = R_6_0;
    H_collection(4,4,i) = 1;
    H_collection(1:3,4,i) = X_baseframe(:,i);
end


q_collection = zeros(6,100);
for i = 1:100
    q_collection(:,i) = inverse_kuka(H_collection(:,:,i), myrobot); %NOTE: myrobot must have been already defined here in the workspace
end

%%
% to run this in a loop:
for i = 1:100
    setAngles(q_collection(:,i), 0.01);
end