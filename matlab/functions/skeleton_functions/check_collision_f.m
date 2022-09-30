function flag = check_collision_f(d_min,d_0,d_start)

    if d_min<(d_0+d_start)
        flag = 1;
    else
        flag = 0;
    end
    
end

