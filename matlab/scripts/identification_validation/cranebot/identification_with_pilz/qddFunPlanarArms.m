function qddk = qddFunPlanarArms(p,qk,qdk,qkRef,qdkRef,expAxis)



    if (expAxis == 'X')

        pCell = num2cell(p);
        qkCell = num2cell(qk);
        qdkCell = num2cell(qdk);

        [qC,qAL1,qAR1,qAL2,qAR2] = deal(qkCell{:});
        [qCd,qAL1d,qAR1d,qAL2d,qAR2d] = deal(qdkCell{:});

        [lCz,mC,fvC,fsC,kd,kp,iA1yy,iA2yy,iCyy] = deal(pCell{:});

        B = B_planarX_arms_f(iA1yy,iA2yy,iCyy,lCz,mC,qAL1,qAL2,qAR1,qAR2);
        n = n_planarX_arms_f(fsC,fvC,lCz,mC,qAL1,qAL2,qAL1d,qAL2d,qAR1,qAR2,qAR1d,qAR2d,qC,qCd);

        Kd = diag([0,kd,kd,kd,kd]);
        Kp = diag([0,kp,kp,kp,kp]);

    elseif (expAxis == 'Y')

        pCell = num2cell(p);
        qkCell = num2cell(qk);
        qdkCell = num2cell(qdk);

        [qC,qAL1,qAR1,qAL2,qAR2] = deal(qkCell{:});
        [qCd,qAL1d,qAR1d,qAL2d,qAR2d] = deal(qdkCell{:});

        [lCz,mC,fvC,fsC,kd,kp,iA1xx,iA2xx,iCxx] = deal(pCell{:});

%         B = B_planar_arms_f(iA1xx,iA2xx,iCxx,lCz,mC,qAL1,qAL2,qAR1,qAR2);
%         n = n_planar_arms_f(fsC,fvC,lCz,mC,qAL1,qAL2,qAL1d,qAL2d,qAR1,qAR2,qAR1d,qAR2d,qC,qCd);

        B = B_planarY_arms_f(iA1xx,iA2xx,iCxx,lCz,mC,qAL1,qAL2,qAR1,qAR2);
        n = n_planarY_arms_f(fsC,fvC,lCz,mC,qAL1,qAL2,qAL1d,qAL2d,qAR1,qAR2,qAR1d,qAR2d,qC,qCd);
        
        Kd = diag([0,kd,kd,kd,kd]);
        Kp = diag([0,kp,kp,kp,kp]);
    elseif (expAxis == 'Z')
        pCell = num2cell(p);
        qkCell = num2cell(qk);
        qdkCell = num2cell(qdk);

        [qC,qAL1,qAR1] = deal(qkCell{:});
        [qCd,qAL1d,qAR1d] = deal(qdkCell{:});

        [iCzz,fvC,fsC,kd,kp,kdZ,kpZ] = deal(pCell{:});

        B = B_planarZ_arms_f(iCzz,qAL1,qAR1);
        n = n_planarZ_arms_f(fsC,fvC,qAL1,qAL1d,qAR1,qAR1d,qCd);

        Kd = diag([kdZ,kd,kd]);
        Kp = diag([kpZ,kp,kp]);
    end

    % Definition command references vectors
    q_des = [zeros(1,1); qkRef];
    qd_des = [zeros(1,1); qdkRef];

    % Inverse dynamics
    qddk = B\(Kd*(qd_des-qdk)+Kp*(q_des-qk)-n);
    
end
