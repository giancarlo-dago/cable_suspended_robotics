function qd = qdFun(k,p,q0,qd0,qdd0,Ts)

    disp('qdFun')
    if k==0
        qd = qd0;
    else
        qd = qdFun(k-1,p,q0,qd0,qdd0,Ts) + qddFun(k-1,p,q0,qd0,qdd0,Ts)*Ts;
    end

end