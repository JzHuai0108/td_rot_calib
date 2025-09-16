function [R, t, s] = umeyama_alignment(X, Y, allow_scale)
% X,Y: 3xN. Solve Y ≈ s R X + t
    if nargin < 3, allow_scale = true; end
    N = size(X,2);
    muX = mean(X,2); muY = mean(Y,2);
    Xc = X - muX;    Yc = Y - muY;

    Sigma = (Yc * Xc.') / N;
    [U, ~, V] = svd(Sigma);
    D = eye(3); if det(U*V') < 0, D(3,3) = -1; end
    R = U * D * V';

    varX = mean(sum(Xc.^2,1));
    if allow_scale
        s = trace(Sigma' * R) / varX;
    else
        s = 1.0;
    end
    t = muY - s * R * muX;
end
