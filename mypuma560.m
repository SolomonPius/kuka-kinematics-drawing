function myrobot = mypuma560(DH)
    L1 = Link( 'd', DH(1,2), 'a', DH(1,3) ,'alpha', DH(1,4));
    L2 = Link( 'd', DH(2,2), 'a', DH(2,3) ,'alpha', DH(2,4));
    L3 = Link( 'd', DH(3,2), 'a', DH(3,3) ,'alpha', DH(3,4));
    L4 = Link( 'd', DH(4,2), 'a', DH(4,3) ,'alpha', DH(4,4));
    L5 = Link( 'd', DH(5,2), 'a', DH(5,3) ,'alpha', DH(5,4));
    L6 = Link('d', DH(6,2), 'a', DH(6,3) ,'alpha', DH(6,4));
    Links = [L1,L2,L3,L4,L5,L6];
    myrobot = SerialLink(Links);
end

