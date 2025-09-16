function matches = associate2(first_list, second_list, offset, max_difference)
% ASSOCIATE2  Greedy 1-1 time association within tolerance.
% first_list, second_list: columns: [timestamp, ...]
% offset is ADDED to second_list timestamps before matching.
% Returns matches as [i_first, i_second] (1-based).

    % Build arrays: [time, src_id(1/2), row_index]
    firstArray  = [first_list(:,1),           ones(size(first_list,1),1),  (1:size(first_list,1))'];
    secondArray = [second_list(:,1)+offset,   2*ones(size(second_list,1),1), (1:size(second_list,1))'];

    resultArray = merge_sorted_arrays_with_sentinel(firstArray, secondArray);

    secondUsed  = false(size(second_list,1),1);
    matches     = zeros(min(size(firstArray,1), size(secondArray,1)), 2);
    matchCount  = 0;

    for i = 1:size(resultArray,1)
        src = resultArray(i,2);
        if src == 1 % from first_list
            idx_first = resultArray(i,3);
            t_first   = firstArray(idx_first,1);

            % Look left neighbor (must be from second and unused)
            leftdiff  = inf; left_j = -1;
            if i>1 && resultArray(i-1,2)==2 && ~secondUsed(resultArray(i-1,3))
                left_j   = resultArray(i-1,3);
                leftdiff = abs(t_first - secondArray(left_j,1));
            end

            % Look right neighbor (must be from second and unused)
            rightdiff = inf; right_j = -1;
            if i<size(resultArray,1) && resultArray(i+1,2)==2 && ~secondUsed(resultArray(i+1,3))
                right_j   = resultArray(i+1,3);
                rightdiff = abs(t_first - secondArray(right_j,1));
            end

            if (leftdiff <= max_difference) || (rightdiff <= max_difference)
                if leftdiff <= rightdiff
                    matchCount = matchCount + 1;
                    matches(matchCount,:) = [idx_first, left_j];
                    secondUsed(left_j) = true;
                else
                    matchCount = matchCount + 1;
                    matches(matchCount,:) = [idx_first, right_j];
                    secondUsed(right_j) = true;
                end
            end
        end
    end

    matches = matches(1:matchCount,:);

end

function A = merge_sorted_arrays_with_sentinel(L, R)
% Merge by time (col 1). L,R are unsorted here; sort first.
    [~, iL] = sort(L(:,1)); Ls = L(iL,:);
    [~, iR] = sort(R(:,1)); Rs = R(iR,:);
    % Classic merge (no need for explicit sentinels)
    m = size(Ls,1); n = size(Rs,1);
    A = zeros(m+n, size(L,2));
    i=1; j=1; k=1;
    while i<=m && j<=n
        if Ls(i,1) <= Rs(j,1)
            A(k,:) = Ls(i,:); i=i+1;
        else
            A(k,:) = Rs(j,:); j=j+1;
        end
        k=k+1;
    end
    while i<=m, A(k,:) = Ls(i,:); i=i+1; k=k+1; end
    while j<=n, A(k,:) = Rs(j,:); j=j+1; k=k+1; end
end
