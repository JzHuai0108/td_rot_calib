function [td, mn, mx] = findTimeOffset(atimetxt, btimetxt, outputdir)
% find time offset between two sensors due to transmission delay,
% assume the two sensors have the same remote sensor clock.
% atimetxt the text file containing sensor a's times
% each line: remote time, local time, corrected local time, gnss time
% btimetxt the text file containing sensor b's times
% each line: remote time, local time, corrected local time, gnss time
% return time offset
% bsensor gnss clock + time offset = asensor gnss clock
addpath('../export_fig');
ta = readmatrix(atimetxt);
tb = readmatrix(btimetxt);

swapped = false;
if size(ta, 1) > size(tb, 1)
    temp = ta;
    ta = tb;
    tb = temp;
    swapped = true;
end

sindex = 1;
gindex = 4;
j0 = 1;
offsets = zeros(size(ta, 1), 1);
status = zeros(size(ta, 1), 1);
for i = 1:size(ta, 1)
    for j = j0 : size(tb, 1)
        if tb(j, sindex) > ta(i, sindex)
            j0 = j;
            break
        end
    end
    if j0 < 2
        j0 = max(1, j0);
        continue
    elseif j == size(tb, 1)
        break
    else
        assert(ta(i, sindex) < tb(j0, sindex));
        assert(ta(i, sindex) >= tb(j0 - 1, sindex));
        left = j0 - 1;
        right = j0;
        if tb(right, sindex) - tb(left, sindex) < 1e-5
            fprintf('Warn: too close sensor times %d:%.6f and %d:%.6f\n', left, tb(left, sindex), right, tb(right, sindex));
            left = j0 - 2;
        end
        asensortime = ta(i, sindex);
        agnsstime = ta(i, gindex);
        leftbs = tb(left, sindex);
        leftbg = tb(left, gindex);
        rightbs = tb(right, sindex);
        rightbg = tb(right, gindex);
        querybs = asensortime;
        ratio = (querybs - leftbs) / (rightbs - leftbs);
        querybg = leftbg + ratio * (rightbg - leftbg);
        offsets(i) = agnsstime - querybg;
        status(i) = 1;
    end
end

validoffsets = offsets(status == 1);
md = median(validoffsets);
mx = max(validoffsets);
mn = min(validoffsets);

if swapped
    temp = ta;
    ta = tb;
    tb = temp;

    temp = mx;
    mx = -mn;
    mn = -temp;
    md = -md;
end
fprintf('median %.6f min %.6f max %.6f\n', md, mn, mx);
td = md;

close all;
figure;
s = ta(1, gindex);
t1 = ta(:, gindex) - s;
da = ta(:, gindex) - ta(:, sindex);

t2 = tb(:, gindex) - s;
db = tb(:, gindex) - tb(:, sindex) + td;
plot(t1, da, 'r+'); hold on;
plot(t2, db, 'g*');
if length(t1) == length(offsets)
    plot(t1, offsets, 'b*');
else
    plot(t2, offsets, 'b*');
end
legend('sensor a gnss time - sensor time', 'sensor b gnss time - sensor time + td', 'time offsets');

if length(outputdir) > 0
    export_fig(fullfile(outputdir, 'offset_topic_times.pdf'));
    fprintf('%s %s %.6f %.6f %.6f %.6f\n', atimetxt, btimetxt, td, mn, mx, mx - mn);
end
end
