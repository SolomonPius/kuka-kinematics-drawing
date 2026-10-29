function myrobot = mykuka(DH)
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