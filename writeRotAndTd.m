function writeRotAndTd(dataname, R, td, td_list, outfile, stdeps)

% Author: Jianzhu Huai
% Date: 2024

fid = fopen(outfile, 'a');
fprintf(fid, '%s,%.9f,%.9f,%.9f,%.9f,%.9f', dataname, td, td_list(1), td_list(2), td_list(3), td_list(4));
for i=1:3
    for j=1:3
        fprintf(fid, ',%.9f', R(i, j));
    end
end

ypr = rotm2eul(R, "ZYX");
% rpy2 = myrotm2eul(R); % same values as the above ypr though reversed.
fprintf(fid, ',%.9f,%.9f,%.9f', ypr(3) * 180 / pi, ypr(2) * 180 / pi, ypr(1) * 180 / pi); % roll pitch yaw
if nargin > 5
    fprintf(fid, ',%.8f,%.8f,%.8f', stdeps(1), stdeps(2), stdeps(3));
end
fprintf(fid, '\n');
fclose(fid);
end

function rpy = myrotm2eul(R)
    roll = atan2(R(3, 2), R(3, 3));
    pitch = asin(-R(3, 1));
    yaw = atan2(R(2, 1), R(1, 1));
    rpy = [roll, pitch, yaw];
end
