function [R, t, s] = umeyama_alignment(X, Y, allow_scale)
% Umeyama (1991) alignment: find R,t,(s) s.t. Y ≈ s*R*X + t.
% X,Y: 3xN
    if nargin < 3, allow_scale = true; end
    if size(X,1) ~= 3 || size(Y,1) ~= 3 || size(X,2) ~= size(Y,2)
        error('umeyama_alignment: X and Y must be 3xN with equal N.');
    end
    N = size(X,2);

    muX = mean(X, 2);
    muY = mean(Y, 2);
    X0  = X - muX;
    Y0  = Y - muY;

    % Covariance (Y * X^T) / N so that R maps X -> Y
    Sigma = (Y0 * X0') / N;

    [U, S, V] = svd(Sigma);
    D = eye(3);
    if det(U*V') < 0
        D(3,3) = -1;
    end
    R = U * D * V';

    if allow_scale
        varX = sum(sum(X0.^2)) / N;             % mean squared norm of X0
        s = trace(S * D) / max(varX, eps);
    else
        s = 1.0;
    end

    t = muY - s * R * muX;
end