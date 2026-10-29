function myrobot = mykuka_search(delta)
    % MYKUKA_SEARCH  KUKA model with tool offsets a6 and d6 perturbed by delta (used for calibration)
    d_1 = 400;
    a_1 = 25;
    a_2 = 315;
    a_3 = 35;
    d_4 = 365;
    a_6 = 296.23 + delta(1);
    d_6 = 161.44 + delta(2);
    DH = [0, d_1, a_1,  pi/2 ;
      0, 0  , a_2,  0    ;
      0, 0  , a_3,  pi/2 ;
      0, d_4, 0,    -pi/2;
      0, 0  , 0,    pi/2 ;
      0, d_6, -a_6, 0     ];
    a = DH(:,3);
    alpha = DH(:,4);
    d = DH(:,2);
    L_1 = Link('d', d(1), 'a', a(1), 'alpha', alpha(1));
    L_2 = Link('d', d(2), 'a', a(2), 'alpha', alpha(2));
    L_3 = Link('d', d(3), 'a', a(3), 'alpha', alpha(3));
    L_4 = Link('d', d(4), 'a', a(4), 'alpha', alpha(4));
    L_5 = Link('d', d(5), 'a', a(5), 'alpha', alpha(5));
    L_6 = Link('d', d(6), 'a', a(6), 'alpha', alpha(6));
    myrobot = SerialLink([L_1, L_2, L_3, L_4, L_5, L_6]);
end
