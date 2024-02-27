function p = load_mu(param_robot, mu)
    p = param_robot;
    
    p.m1 = mu(1);
    p.sp1x = mu(2);
    p.sp1y = mu(3);
    p.sp1z = mu(4);
    p.I1xx = mu(5);
    p.I1xy = mu(6);
    p.I1xz = mu(7);
    p.I1yy = mu(8);
    p.I1yz = mu(9);
    p.I1zz = mu(10);
    p.FV1 = mu(11);
    p.FS1 = mu(12);
       
    p.m2 = mu(13);
    p.sp2x = mu(14);
    p.sp2y = mu(15);
    p.sp2z = mu(16);
    p.I2xx = mu(17);
    p.I2xy = mu(18);
    p.I2xz = mu(19);
    p.I2yy = mu(20);
    p.I2yz = mu(21);
    p.I2zz = mu(22);
    p.FV2 = mu(23);
    p.FS2 = mu(24);
    
    p.m3 = mu(25);
    p.sp3x = mu(26);
    p.sp3y = mu(27);
    p.sp3z = mu(28);
    p.I3xx = mu(29);
    p.I3xy = mu(30);
    p.I3xz = mu(31);
    p.I3yy = mu(32);
    p.I3yz = mu(33);
    p.I3zz = mu(34);
    p.FV3 = mu(35);
    p.FS3 = mu(36);
    
    p.m4 = mu(37);
    p.sp4x = mu(38);
    p.sp4y = mu(39);
    p.sp4z = mu(40);
    p.I4xx = mu(41);
    p.I4xy = mu(42);
    p.I4xz = mu(43);
    p.I4yy = mu(44);
    p.I4yz = mu(45);
    p.I4zz = mu(46);
    p.FV4 = mu(47);
    p.FS4 = mu(48);
    
    p.m5 = mu(49);
    p.sp5x = mu(50);
    p.sp5y = mu(51);
    p.sp5z = mu(52);
    p.I5xx = mu(53);
    p.I5xy = mu(54);
    p.I5xz = mu(55);
    p.I5yy = mu(56);
    p.I5yz = mu(57);
    p.I5zz = mu(58);
    p.FV5 = mu(59);
    p.FS5 = mu(60);
    
    p.m6 = mu(61);
    p.sp6x = mu(62);
    p.sp6y = mu(63);
    p.sp6z = mu(64);
    p.I6xx = mu(65);
    p.I6xy = mu(66);
    p.I6xz = mu(67);
    p.I6yy = mu(68);
    p.I6yz = mu(69);
    p.I6zz = mu(70);
    p.FV6 = mu(71);
    p.FS6 = mu(72);
    
    p.m7 = mu(73);
    p.sp7x = mu(74);
    p.sp7y = mu(75);
    p.sp7z = mu(76);
    p.I7xx = mu(77);
    p.I7xy = mu(78);
    p.I7xz = mu(79);
    p.I7yy = mu(80);
    p.I7yz = mu(81);
    p.I7zz = mu(82);
    p.FV7 = mu(83);
    p.FS7 = mu(84);
end