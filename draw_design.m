% X_workspace is a 3xn matrix. It represents the paths to be drawn on the paper in the paper frame.
% We want to draw the pattern in the .xlsx file. The Z will remain constant, but X and Y will change.
% Jug design: (x, y) points in the paper frame, scaled 10x and offset 550 mm in x
data = xlsread(fullfile('designs','jug.xlsx'));
x_val_workspace = 550 + 10*data(:,1);
y_val_workspace = 10*data(:,2);
%%

n = length(data);

X_workspace =   [x_val_workspace.';
                 y_val_workspace.';
                 -3.*ones(1, length(data))];

% X_baseframe is a 3xn matrix with the same points as X_workspace but expressed in workspace coordinates
X_baseframe = zeros(3, n);
for i = 1:n
    % Update each column with the FrameTransformation function
    X_baseframe(:,i) = FrameTransformation(X_workspace(:,i));
end

% Reconstruct each H matrix
% H_collection is a 4x4xn matrix, a collection of 100 4x4 matrices
H_collection = zeros(4,4,n);
R_6_0 = [0, 0,  1;
         0, -1, 0;
         1, 0,  0];
for i = 1:n
    H_collection(1:3,1:3,i) = R_6_0;
    H_collection(4,4,i) = 1;
    H_collection(1:3,4,i) = X_baseframe(:,i);
end


q_collection = zeros(6,n);
for i = 1:n
    q_collection(:,i) = inverse_kuka(H_collection(:,:,i), myrobot); %NOTE: myrobot must have been already defined here in the workspace
end

%%
% to run this in a loop:
for i = 1:n
    setAngles(q_collection(:,i), 0.01);
end

