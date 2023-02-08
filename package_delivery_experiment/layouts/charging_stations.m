% This file returns the  left-bottom and right-top points of charging
% stations
function [left_bottoms, right_tops] = charging_stations(num_stations)
% First station is specified by : 
% |x <= 2| and |y <= 2| and |x >= 1| and |y >= 1|

max_num_stations = 2;

if num_stations > max_num_stations
    disp('Too many charging stations. Limit the number of stations.');
    num_stations = max_num_stations;
end

if num_stations == 1
    left_bottoms = [0,0];
    right_tops = [1,1];
elseif num_stations == 2
    left_bottoms = [0,0; 9,0];
    right_tops = [1,1; 10,1];
end

end

