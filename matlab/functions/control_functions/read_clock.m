function clock = read_clock(clock_latest_mex)
    
    clock = rosmessage('rosgraph_msgs/Clock');
    clock_sec = clock_latest_mex.Clock_.Sec;
    clock_nsec = clock_latest_mex.Clock_.Nsec;
    
    clock = clock_sec + clock_nsec/1e9;

end