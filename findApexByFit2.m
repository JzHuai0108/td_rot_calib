function [maxvalx, maxvaly] = findApexByFit2(x, y)
% if the maxvalx is 
% input example
% x = [-1, 0, 1]';
% y = - 2*(x - 0.8).^2 + 1;
% output: [0.8, 1]

f = fit(x, y, 'poly2');
c = coeffvalues(f);
cd = polyder(c);
maxvalx = roots(cd);
maxvaly = f(maxvalx);
end