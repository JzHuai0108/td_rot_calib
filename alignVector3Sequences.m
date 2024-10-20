function [time_offset, delay] = alignVector3Sequences(...
        trainData, queryData, interestCols, trainsensor_R_querysensor, ...
        maxLagSecs, outputpath, tsmoothwinsize, qsmoothwinsize)
% find the time offset between two sequences of vector3 data, e.g., angular rates.
% trainData, data in the train sensor frame.
% each line: time[sec], gx, gy, gz[rad/sec], ax, ay, az[m/s^2]
% queryData, data in the query sensor frame.
% each line: time[sec], gx, gy, gz[rad/sec], ax, ay, az[m/s^2]
% interestCols, which columns to consider, e.g., 2:4
% trainsensor_R_querysensor: a rough rotation from query sensor frame to the train sensor.
% return the time offset in seconds. query data original time + time_offset = query data time in train sensor clock.

% Author: Jianzhu Huai
% Date: 2014

format longg
time_alignment_config.smoothing_kernel_size_A = 25;
time_alignment_config.clipping_percentile_A = 99.5;
time_alignment_config.smoothing_kernel_size_B = 25;
time_alignment_config.clipping_percentile_B = 99.0;

if ~exist('tsmoothwinsize', 'var')
    tsmoothwinsize = 1;
end
if ~exist('qsmoothwinsize', 'var')
    qsmoothwinsize = 1;
end

trainData=removeRepetitiveEntriesAndReorder(trainData);
time_ref = trainData(1,1);
queryData(:, 1) = queryData(:, 1) - time_ref;
trainData(:, 1) = trainData(:, 1) - time_ref;

queryDataSlice = [queryData(:, 1), queryData(:, interestCols)*trainsensor_R_querysensor'];
trainDataSlice = trainData(:, [1, interestCols]);

% make the query seq starts at a close time as train seq
qi = 1;
while queryDataSlice(qi, 1) < trainDataSlice(1, 1)
    qi = qi + 1;
end
queryDataSlice = queryDataSlice(qi:end, :);
ti = 1;
while trainDataSlice(ti, 1) < queryDataSlice(1, 1)
    ti = ti + 1;
end
trainDataSlice = trainDataSlice(ti:end, :);
fprintf('Start alignment culled %d train and %d query entries\n', ti - 1, qi - 1);
% also cull the end
if trainDataSlice(end, 1) > queryDataSlice(end, 1) + maxLagSecs
    lastti = find(trainDataSlice(:, 1) > queryDataSlice(end, 1) + maxLagSecs, 1);
    trainDataSlice = trainDataSlice(1:lastti-1, :);
elseif queryDataSlice(end, 1) > trainDataSlice(end, 1) + maxLagSecs
    lastqi = find(queryDataSlice(:, 1) > trainDataSlice(end, 1) + maxLagSecs, 1);
    queryDataSlice = queryDataSlice(1:lastqi-1, :);
end

% find out the data sample interval to decide the step in sampling timeline
trainSampleInterval = trainDataSlice(2:end,1) - trainDataSlice(1:end-1,1);
trainMedianSampleInterval = median(trainSampleInterval);
queryMedianSampleInterval = median(diff(queryDataSlice(:,1)));
if queryMedianSampleInterval > 5 * trainMedianSampleInterval
    tsmoothwinsize = max(5, tsmoothwinsize);
elseif trainMedianSampleInterval > 5 * queryMedianSampleInterval
    qsmoothwinsize = max(5, qsmoothwinsize);
end
fprintf('Train sample interval %.3f, query sample interval %.3f\n', trainMedianSampleInterval, queryMedianSampleInterval);
fprintf('Setting smooth win size for train to %d, for query to %d\n', tsmoothwinsize, qsmoothwinsize);

time_step= min(trainMedianSampleInterval, queryMedianSampleInterval);
if(max(trainSampleInterval)> 10*time_step)
    fprintf('Warning: Too large gap in timestamps of train samples, train: %f, query: %f\n', max(trainSampleInterval),max(queryMedianSampleInterval) );
end
fprintf('Correlation time step/resolution: %f sec\n', time_step);

%% before alignment
figure;
draw3axis(trainDataSlice, queryDataSlice, 'time w.r.t the train data start[sec]', 'Train and query quantities before time alignment');
if ~isempty(outputpath)
    saveas(gcf, [outputpath, filesep, 'original_sequences.pdf']);
end

%% before alignment zoom
if trainDataSlice(end, 1) - trainDataSlice(1, 1) > 60
    figure;
    id25 = find(trainDataSlice(:, 1) > trainDataSlice(1, 1) + 25, 1);
    id50 = find(trainDataSlice(:, 1) > trainDataSlice(1, 1) + 50, 1);
    id25q = find(queryDataSlice(:, 1) > trainDataSlice(1, 1) + 25, 1);
    id50q = find(queryDataSlice(:, 1) > trainDataSlice(1, 1) + 50, 1);

    draw3axis(trainDataSlice(id25:id50, :), queryDataSlice(id25q:id50q, :), ...
        'time w.r.t the train data start[sec]', 'Zoomed train and query quantities before time alignment');
    if ~isempty(outputpath)
        saveas(gcf, [outputpath, filesep, 'original_sequences_zoomed.pdf']);
    end
end


% remove mean values
queryDataSliceNoBias = queryDataSlice;
trainDataSliceNoBias = trainDataSlice;
for jack=2:4
    queryDataSliceNoBias(:,jack) = queryDataSlice(:,jack)- mean(queryDataSlice(:,jack));
    trainDataSliceNoBias(:,jack) = trainDataSlice(:,jack)- mean(trainDataSlice(:,jack));
end

% add the dimension of magnitude for comparison
queryDataMag = sqrt(sum(queryDataSlice(:, 2:4).^2, 2));
queryDataMean = mean(queryDataMag);
queryDataMag = queryDataMag - queryDataMean;
queryDataSliceNoBias = [queryDataSliceNoBias, queryDataMag];
columnLabels = {'time', 'x', 'y', 'z', 'norm'};

trainDataMag = sqrt(sum(trainDataSlice(:, 2:4).^2, 2));
trainDataMagMean = mean(trainDataMag);
trainDataMag = trainDataMag - trainDataMagMean;
trainDataSliceNoBias = [trainDataSliceNoBias, trainDataMag];

if tsmoothwinsize > 1
for i=2:size(trainDataSliceNoBias, 2)
    smoothTrain = smoothdata(trainDataSliceNoBias(:, i), 'gaussian', tsmoothwinsize);
    trainDataSliceNoBias(:, i) = smoothTrain;
end
end
if qsmoothwinsize > 1
for i=2:size(queryDataSliceNoBias, 2)
    smoothQuery = smoothdata(queryDataSliceNoBias(:, i), 'gaussian', qsmoothwinsize);
    queryDataSliceNoBias(:, i) = smoothQuery; 
end
end

queryDeltat = trainDataSliceNoBias(1, 1) - queryDataSliceNoBias(1, 1);

query_sample_times = queryDataSliceNoBias(1, 1):time_step:queryDataSliceNoBias(end, 1);
sample_times = trainDataSlice(1, 1):time_step:trainDataSlice(end, 1);
datadims = size(queryDataSliceNoBias, 2)-1;
delay = zeros(datadims,1);

figure;
for jack =2:size(queryDataSliceNoBias, 2)
    v3query = cubicfit(queryDataSliceNoBias(:, 1)', queryDataSliceNoBias(:, jack)', query_sample_times);
    [v3train, ~, ~]=cubicfit(trainDataSliceNoBias(:, 1)', trainDataSliceNoBias(:, jack)', sample_times);
    [number_delay, maxcorr, lags, xc] = estimateDelayOfSensorReading(v3query, v3train, ceil(queryDeltat + maxLagSecs / time_step));
    lags = lags * time_step;
    delay(jack-1) = - number_delay*time_step + queryDeltat + (qsmoothwinsize - tsmoothwinsize) * time_step / 2;
    fprintf('Query data clock delays by %.6f sec to train data clock on %s\n', delay(jack-1), columnLabels{jack});
    subplot(datadims,1,jack-1);
    plot(lags, xc); hold on;
    plot(number_delay*time_step, maxcorr, 'r*');
    grid on;
    ylabel(columnLabels{jack});
    msgx = sprintf('Query sensor timestamps corrects to train sensor clock by max corr at %.3f (ms)', -number_delay*time_step*1000);
    xlabel(msgx);
    xlim([-1, 1] + number_delay*time_step);
    if ~isempty(outputpath)
    saveas(gcf, [outputpath, filesep, 'correlation.pdf']);
    end
end

%% fit linear drift
dominantaxis = 4;
time_offset = delay(dominantaxis);
fprintf('The local time w.r.t the train clock for an entry in the query sensor = \n');
fprintf('timestamp of the entry + time offset %.6f sec +/- %.6f sec.\n', time_offset, time_step);
%% after shift time
figure;
qdshifted = queryDataSlice;
qdshifted(:, 1) = qdshifted(:, 1) + time_offset;
draw3axis(trainDataSlice, qdshifted, 'time w.r.t the train data start[sec]', 'Train and query quantities after time alignment');
if ~isempty(outputpath)
saveas(gcf, [outputpath, filesep, 'aligned_sequences.pdf']);
end

%% after alignment zoom
if trainDataSlice(end, 1) - trainDataSlice(1, 1) > 60
    figure;
    id25 = find(trainDataSlice(:, 1) > trainDataSlice(1, 1) + 25, 1);
    id50 = find(trainDataSlice(:, 1) > trainDataSlice(1, 1) + 50, 1);
    id25q = find(qdshifted(:, 1) > trainDataSlice(1, 1) + 25, 1);
    id50q = find(qdshifted(:, 1) > trainDataSlice(1, 1) + 50, 1);

    draw3axis(trainDataSlice(id25:id50, :), qdshifted(id25q:id50q, :), ...
        'time w.r.t the train data start[sec]', 'Zoomed train and query quantities after time alignment');
    if ~isempty(outputpath)
        saveas(gcf, [outputpath, filesep, 'aligned_sequences_zoomed.pdf']);
    end
end

end

function draw3axis(td, qd, xlabelstr, titlestr)
ax1 = subplot(3,1,1);
plot(td(:,1), td(:,2), 'r'); hold on;
plot(qd(:,1), qd(:, 2), '--b');
title(titlestr);
legend('train x', 'query x');
grid on; ylabel('x');
ax2 = subplot(3,1,2);
plot(td(:,1), td(:,3), 'r'); hold on;
plot(qd(:,1), qd(:, 3), '--b');
legend('train y', 'query y');
grid on; ylabel('y');
ax3 = subplot(3,1,3);
plot(td(:,1), td(:,4), 'r'); hold on;
plot(qd(:,1), qd(:, 4), '--b');
legend('train z', 'query z');
grid on; ylabel('z');
xlabel(xlabelstr, 'Interpreter', 'none');
linkaxes([ax1,ax2,ax3],'xy');
end
