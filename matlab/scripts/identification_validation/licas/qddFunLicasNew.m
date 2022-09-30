function qddk = qddFunLicasNew(p,qk,qdk,qkRef,qdkRef)

    kdL1 = 0.55;
    kdL2 = 0.5;
    kdL3 = 0.1;
    kdL4 = 0.55;
    kdR1 = 0.55;
    kdR2 = 0.5;
    kdR3 = 0.1;
    kdR4 = 0.55;
    kpL1 = 200;
    kpL2 = 50;
    kpL3 = 1.5;
    kpL4 = 50;
    kpR1 = 200;
    kpR2 = 50;
    kpR3 = 1.5;
    kpR4 = 50;
    kdSx = 1;
    KdSy = 1;
    kpSx = 700;
    kpSy = 700;
    
    pCell = num2cell(p);
    qkCell = num2cell(qk);
    qdkCell = num2cell(qdk);

    L = 1.0;

    m4 = 0.6390;
    m5 = 0.6390;
    m6 = 0.6390;
    
    [m3,fv3,l3,iyy,   m2,fv2,l2,ixx,   fv1,izz,kpZ,kdZ,    kpSz,kdSz] = deal(pCell{:});


    [q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14] = deal(qkCell{:});
    [qd1,qd2,qd3,qd4,qd5,qd6,qd7,qd8,qd9,qd10,qd11,qd12,qd13,qd14] = deal(qdkCell{:});
    Fv = diag([0 0 0 0 0 0 0 0 0 0 0 0 0 0]);

    B = BNew_f(L,ixx,iyy,izz,l2,l3,m2,m3,m4,m5,m6,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14);
    n = nNew_f(L,fv1,fv2,fv3,ixx,iyy,izz,l2,l3,m2,m3,m4,m5,m6,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,qd1,qd2,qd3,qd4,qd5,qd6,qd7,qd8,qd9,qd10,qd11,qd12,qd13,qd14) + Fv*qdk;
    
    Kd = diag([kdZ,0,0,kdSz,kdSx,KdSy,kdL1,kdL2,kdL3,kdL4,kdR1,kdR2,kdR3,kdR4]);
    Kp = diag([kpZ,0,0,kpSz,kpSx,kpSy,kpL1,kpL2,kpL3,kpL4,kpR1,kpR2,kpR3,kpR4]);

    % Definition command references vectors
    q_des = [zeros(4,1); -q2; -q3; qkRef];
    qd_des = [zeros(6,1); qdkRef];

    % Inverse dynamics
    qddk = inv(B)*(Kd*(qd_des-qdk)+Kp*(q_des-qk)-n);

end

