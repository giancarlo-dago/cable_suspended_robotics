function qddk = qddFunCranebotPilz(p,qk,qdk,qkRef,qdkRef)

    pCell = num2cell(p);
    qkCell = num2cell(qk);
    qdkCell = num2cell(qdk);

%     [L, massCablesPulleys, lCablesPulleys, ixxCablesPulleys, iyyCablesPulleys, izzCablesPulleys,...
%         fricCablesJz, fricCablesJx, fricCablesJy, fricPlatformJz] = deal(pCell{:});

    [L, massCablesPulleys, lCablesPulleys, ixxCablesPulleys, ...
        dampCablesJx, fricCablesJx] = deal(pCell{:});
   
    [q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13] = deal(qkCell{:});
    [qd1,qd2,qd3,qd4,qd5,qd6,qd7,qd8,qd9,qd10,qd11,qd12,qd13] = deal(qdkCell{:});

    B = B_CranebotPilzPlanarUpperX_f(L,ixxCablesPulleys,lCablesPulleys,massCablesPulleys,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13);

    n = n_CranebotPilzPlanarUpperX_f(L,dampCablesJx,fricCablesJx,lCablesPulleys,massCablesPulleys,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,qd1,qd2,qd3,qd4,qd5,qd6,qd7,qd8,qd9,qd10,qd11,qd12,qd13);


    Kd = diag([0,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5]);
    Kp = diag([0,50,50,50,50,50,50,50,50,50,50,50,50]);

    % Definition command references vectors
    q_des = [zeros(1,1); qkRef];
    qd_des = [zeros(1,1); qdkRef];

    % Inverse dynamics
    qddk = B\(Kd*(qd_des-qdk)+Kp*(q_des-qk)-n);
    
end
