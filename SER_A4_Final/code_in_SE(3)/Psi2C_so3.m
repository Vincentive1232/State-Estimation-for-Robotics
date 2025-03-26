function C = Psi2C_so3(Psi)
    % This is used for transforming the rotational vector into a rotational
    % matrix
    Psi_norm = norm(Psi);
    Psi_v_norm = Psi./Psi_norm;
    Id3 = eye(3);

    C = cos(Psi_norm)*Id3 + (1 - cos(Psi_norm))*(Psi_v_norm*Psi_v_norm') - sin(Psi_norm)*v2skew(Psi_v_norm);
end