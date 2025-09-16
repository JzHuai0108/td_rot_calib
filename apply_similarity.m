function P_aligned = apply_similarity(P_est_all, R, t, s)
% P_est_all: Nx3 (all est positions)
% Returns Nx3 aligned positions
P_aligned = (s * (R * P_est_all')).' + t.';
end
