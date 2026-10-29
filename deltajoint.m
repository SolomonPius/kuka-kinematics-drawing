function q = deltajoint(delta)
    % DELTAJOINT  Calibration cost function for fminunc.
    % Returns the summed position error at the three calibration points for a given perturbation delta
    % delta is a 2 element vector containing delta_1 and delta_2, which are perturbations to a_6 and d_6 respectively

    % create a SerialLinks 'robot' instance with the perturbations
    kuka = mykuka_search(delta);

    %-------------------------- Calibration ----------------------------%
    % Measured calibration points: Xi are 3x1 pencil positions (mm),
    % Qi are the 1x6 joint angles recorded at each point (rad)
    X1 = [675.71; 179.56; 28.02];
    X2 = [681.04; -148.32; 24.14];
    X3 = [620.14; -319.63; 25.31];
    Q1 = [0.2492    0.6360    0.0428    0.0255    0.8954   -0.0161];
    Q2 = [-0.2250    0.6302    0.0429    0.0255    0.8954   -0.0161];
    Q3 = [   -0.4864    0.6320    0.0429    0.0255    0.8954   -0.0161];
    

    H1=forward_kuka(Q1, kuka);
    H2=forward_kuka(Q2, kuka);
    H3=forward_kuka(Q3, kuka);
    
    q=norm(H1(1:3,4)-X1)+norm(H2(1:3,4)-X2)+norm(H3(1:3,4)-X3);
end