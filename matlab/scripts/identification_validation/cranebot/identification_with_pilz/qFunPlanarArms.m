function [qk, qdk] = qFunPlanarArms(p,qPrev,qdPrev,Ts,qPrevRef,qdPrevRef,expAxis)

    qPrev = qPrev';                                                 % Transposition for size compliance
    qdPrev = qdPrev';                                               % Transposition for size compliance
    qPrevRef = qPrevRef';                                           % Transposition for size compliance
    qdPrevRef = qdPrevRef';                                         % Transposition for size compliance
    qddPrev = qddFunPlanarArms(p,qPrev,qdPrev,qPrevRef,qdPrevRef,expAxis);    % Compute qdd(k-1) from q(k-1) and qd(k-1)
    qdk = qdPrev + qddPrev*Ts;                                      % Integrate to get qd(k)
    qk = qPrev + qdPrev*Ts;                                         % Integrate to get q(k)

end
