function [qk, qdk] = qFunPlanar(p,qPrev,qdPrev,Ts,qPrevRef,qdPrevRef)

    qPrev = qPrev';                                                 % Transposition for size compliance
    qdPrev = qdPrev';                                               % Transposition for size compliance
    qPrevRef = qPrevRef';                                           % Transposition for size compliance
    qdPrevRef = qdPrevRef';                                         % Transposition for size compliance
    qddPrev = qddFunPlanar(p,qPrev,qdPrev,qPrevRef,qdPrevRef);    % Compute qdd(k-1) from q(k-1) and qd(k-1)
    qdk = qdPrev + qddPrev*Ts;                                      % Integrate to get qd(k)
    qk = qPrev + qdPrev*Ts;                                         % Integrate to get q(k)

end
