% This file returns the  left-bottom and right-top points of surveillance
% regions
function [left_bottoms, right_tops] = surveillance_spots(num_regions)

max_num_regions = 2;

if num_regions > max_num_regions
    disp('Too many surveillance regions. Limit the number of regions to 2.');
    num_regions = max_num_regions;
end

if num_regions == 1
    left_bottoms = [1.5,5.5];
    right_tops = [4.5,8.5];
elseif num_regions == 2
%     left_bottoms = [1,6; 6,6];
%     right_tops = [5,10; 10,10];
    left_bottoms = [0,7; 6,7];
    right_tops = [4,11; 10,11];
end

end

