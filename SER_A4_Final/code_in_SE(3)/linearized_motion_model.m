function res = linearized_motion_model(T, v, dt)
    res = expm(dt*adjoint(v));
end