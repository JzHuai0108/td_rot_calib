function [valueY, primeDeriv, secondDeriv]=cubicfit(X,Y,x)
% compute values at interpolation points x given known point positions X
% and their values Y. X,Y,x, valueY, etc are 1xN vectors.

% Author: Jianzhu Huai
% Date: 2014

S = spline(X,Y);
% Multiplying the coeffiecients by this matrix M is equivalent to
% differentiation
M = diag(3:-1:1,1);

% First derivative
S1 = S;
S1.coefs = S1.coefs*M;
% Second derivative
S2 = S1;
S2.coefs = S2.coefs*M;

subDim=1;

valueY=ppval(S,x);
valueY=valueY(subDim,:);

primeDeriv=ppval(S1,x);
primeDeriv=primeDeriv(subDim,:);

secondDeriv=ppval(S2,x);
secondDeriv=secondDeriv(subDim,:);
end