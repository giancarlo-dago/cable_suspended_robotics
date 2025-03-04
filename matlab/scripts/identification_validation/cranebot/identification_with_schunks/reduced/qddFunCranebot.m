function qddk = qddFunCranebot(p,qk,qdk,qkRef,qdkRef)

    pCell = num2cell(p);
    qkCell = num2cell(qk);
    qdkCell = num2cell(qdk);

    [L,m_cables,l_cables,ixx_cables,fv1p,fv2p] = deal(pCell{:});
%     [L,m_cables,l_cables,l_platform,ixx_cables,fv1p,fv2p] = deal(pCell{:});
   
    [q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14] = deal(qkCell{:});
    [qd1,qd2,qd3,qd4,qd5,qd6,qd7,qd8,qd9,qd10,qd11,qd12,qd13,qd14] = deal(qdkCell{:});
    
    B = B_reduced_f(L,ixx_cables,l_cables,m_cables,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14);
    n = n_reduced_f(L,fv1p,fv2p,l_cables,m_cables,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,qd1,qd2,qd3,qd4,qd5,qd6,qd7,qd8,qd9,qd10,qd11,qd12,qd13,qd14);

%     B = B_reduced_bis_f(L,ixx_cables,l_cables,l_platform,m_cables,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14);
%     n = n_reduced_bis_f(L,fv1p,fv2p,l_cables,l_platform,m_cables,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,qd1,qd2,qd3,qd4,qd5,qd6,qd7,qd8,qd9,qd10,qd11,qd12,qd13,qd14);

    Kd = diag([0,0,.5,.5,.5,.5,.5,.5,.5,.5,.5,.5,.5,.5]);
    Kp = diag([0,0,50,50,50,50,50,50,50,50,50,50,50,50]);

    % Definition command references vectors
    q_des = [zeros(2,1); qkRef];
    qd_des = [zeros(2,1); qdkRef];

    % Inverse dynamics
    qddk = inv(B)*(Kd*(qd_des-qdk)+Kp*(q_des-qk)-n);
    
end
