function orderedData=removeRepetitiveEntriesAndReorder(rawData)
% remove reptitive entries in the raw data, whose first column is time
% and reorder them in ascending order of time.

% Author: Jianzhu Huai
% Date: 2014

timestamps= rawData(:,1);
[C, ia, ~]= unique(timestamps);
[B, ic] = sort(C);
uniqueData = rawData(ia,:);
orderedData = uniqueData(ic,:);
end
