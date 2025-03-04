function qddk = qddFunPlanar(p,qk,qdk,qkRef,qdkRef)

    pCell = num2cell(p);
    qkCell = num2cell(qk);
    qdkCell = num2cell(qdk);

    [lCz,mC,fvC,fsC] = deal(pCell{:});
   
    [qC,qAL,qAR] = deal(qkCell{:});
    [qCd,qALd,qARd] = deal(qdkCell{:});

    B = B_planar_f(lCz,mC,qAL,qAR);
    n = n_planar_f(fsC,fvC,lCz,mC,qAL,qALd,qAR,qARd,qC,qCd);

    Kd = diag([0,0.5,0.5]);
    Kp = diag([0,50,50]);

    % Definition command references vectors
    q_des = [zeros(1,1); qkRef];
    qd_des = [zeros(1,1); qdkRef];

    % Inverse dynamics
    qddk = B\(Kd*(qd_des-qdk)+Kp*(q_des-qk)-n);
    
end
