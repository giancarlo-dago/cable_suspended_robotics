function qddk = qddFunCranebotComplete(p,qk,qdk,qkRef,qdkRef)

    pCell = num2cell(p);
    qkCell = num2cell(qk);
    qdkCell = num2cell(qdk);

    [L,m_cables,l_cables,ixx_cables,fv1p,fv2p] = deal(pCell{:});
    iyy_cables = ixx_cables;
    izz_cables = 0;
    m_cables1 = m_cables;
    m_cables2 = m_cables;
    l_cables_1 = l_cables;
    l_cables_2 = l_cables;
	fv1p_1 = fv1p;
    fv1p_2 = fv1p;
    fv2p_1 = fv2p;
    fv2p_2 = fv2p;

%     [L,m_cables1,m_cables2,l_cables_1,l_cables_2,ixx_cables,iyy_cables,fv1p_1,fv1p_2,fv2p_1,fv2p_2] = deal(pCell{:});
   
    [q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16] = deal(qkCell{:});
    [qd1,qd2,qd3,qd4,qd5,qd6,qd7,qd8,qd9,qd10,qd11,qd12,qd13,qd14,qd15,qd16] = deal(qdkCell{:});
    
    B = B_complete_f(L,ixx_cables,iyy_cables,izz_cables,l_cables_1,l_cables_2,m_cables1,m_cables2,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16);
    n = n_complete_f(L,fv1p_1,fv1p_2,fv2p_1,fv2p_2,ixx_cables,iyy_cables,izz_cables,l_cables_1,l_cables_2,m_cables1,m_cables2,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,qd1,qd2,qd3,qd4,qd5,qd6,qd7,qd8,qd9,qd10,qd11,qd12,qd13,qd14,qd15,qd16);

    Kd = diag([0,0,0,0,.5,.5,.5,.5,.5,.5,.5,.5,.5,.5,.5,.5]);
    Kp = diag([0,0,0,0,50,50,50,50,50,50,50,50,50,50,50,50]);

    % Definition command references vectors
    q_des = [zeros(4,1); qkRef];
    qd_des = [zeros(4,1); qdkRef];

    % Inverse dynamics
    qddk = inv(B)*(Kd*(qd_des-qdk)+Kp*(q_des-qk)-n);
    
end
