function [number_delay, maxcorr, lags, xc] = ...
    estimateDelayOfSensorReading(meas_sensor, meas_ref, maxlag)
% this function assumes the 1D measurements from the sensor is delayed 
% w.r.t the reference, and input are having sizes 1XN, 1XM
%
% test
% x = randn(1, 100);
% y = [zeros(1, 10), x];
% number_delay=estimateDelayOfSensorReading(y, x, 20)
% assert(number_delay == 10);

% Author: Jianzhu Huai
% Date: 2014

if nargin < 3
    maxlag = max(length(meas_sensor), length(meas_ref));
end
[xc, lags] = xcorr(meas_sensor, meas_ref, maxlag);
[v1, index] = max(xc);
[v2, index2] = max(xc([1:index - 15, index + 15:end]));
% if v1 < v2 * 1.1
%     fprintf(['Warn: there may be a second correlation peak at %d (val %.3f)' ...
%         ' beside %d of val %.3f, ratio %.3f\n'], ...
%         index2, v2, index, v1, v1 / v2);
%     index = index2;
% end

number_delay=lags(index);
maxcorr = xc(index);
% use quadratic interpolation
if index > 1 && index < length(lags)
    [lagatmax, corratmax] = findApexByFit2(...
        lags(index + (-1:1))' - lags(index), xc(index + (-1:1))');
    lagatmax = lagatmax + lags(index);
    if lagatmax > lags(index - 1) && lagatmax < lags(index + 1)
        %  fprintf(['Updated discrete lag number %.3f to %.3f and ', ...
        %            'maxcorr %.4f to %.4f by fit2\n'], number_delay, ...
        %            lagatmax, maxcorr, corratmax);
        number_delay = lagatmax;
        maxcorr = corratmax;
    end
end

end