function P_out = apply_similarity(P_in, R, t, s)
% P_in: Nx3; R:3x3; t:3x1; s:scalar
    P_out = (s * (R * P_in') + t)';  % Nx3
end
