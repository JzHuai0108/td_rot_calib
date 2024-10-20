function [m_R_e, matchesME]= estimateRelativeRotation(imuDataM, imuDataE, channels, maxTimeDiff)
% estimate the relative rotation between two imu sensors E and M
% both imuDataE and imuDataM may contain accelerometer and/or gyro data
% the first column of imuDataE and imuDataM are timestamps
% channels represent which data channel are used for svd decomposition,e.g,
% 2:4 or 5:7
% return estimated rotation from E sensor frame to M sensor frame
% and the corresponding indices of time-wise matched entries

% Author: Jianzhu Huai
% Date: 2014

imuSampleIntervalE = imuDataE(2:end,1) -imuDataE(1:end-1,1);
imuSampleIntervalM = imuDataM(2:end,1) -imuDataM(1:end-1,1);
medianSampleIntervalE = median(imuSampleIntervalE);
medianSampleIntervalM = median(imuSampleIntervalM);
if nargin < 4
    maxTimeDiff = min(medianSampleIntervalE, medianSampleIntervalM)/3;
end
matchesME=associate2(imuDataM, imuDataE, 0, maxTimeDiff);
if(size(matchesME,1)<10)
    fprintf('Not enough matched data %d to estimate relative rotation!\n', size(matchesME, 1));
    m_R_e=[];
    return;
else
    fprintf(['Associated pairs %d with time tol %.6f computed from median ' ...
        'of sample E %.6f and median of sample M %.6f.\n'], size(matchesME, 1), ...
        maxTimeDiff, medianSampleIntervalE, medianSampleIntervalM);
end
% use SVD to obtain the rotation from gyro measurements
Hmat=zeros(3);
for count=1:size(matchesME,1)
    we=imuDataE(matchesME(count,2), channels);
    wm=imuDataM(matchesME(count,1), channels);
    Hmat=Hmat+we'*wm;
end
[U, S, V]=svd(Hmat);
m_R_e=(U*V')';
end