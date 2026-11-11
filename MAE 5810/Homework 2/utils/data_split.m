function [training, testing] = data_split(data, trainpercent, axis)
    [numrows, numcols] = size(data);

    if axis == 1, shuffleidx = randperm(numrows);
    else, shuffleidx = randperm(numcols); end

    splitidx = floor((axis == 1)*numrows*trainpercent + ...
        (axis ~= 1)*numcols*trainpercent); % where to split
    if axis == 1 % if splitting across rows
        training = data(shuffleidx(1:splitidx),:);
        testing = data(shuffleidx(splitidx+1:end), :);
    else % if splitting across columns
        training = data(:,shuffleidx(1:splitidx));
        testing = data(:,shuffleidx(splitidx+1:end));
    end
end