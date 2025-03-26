function res = motion_model(T, v, dt)
    res = expm(dt*v2skew_se3(v))*T;
end