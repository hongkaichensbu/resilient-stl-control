% This file computes the maximum resilience set of an non-empty set
function [set_out, set_idx] = max_res_set(set_in)
% set_in: m-by-2
% m: number of pairs
assert(~isempty(set_in)); % check set_in is not empty

N = size(set_in, 1); 
set_out = []; % the final maximum res set
set_idx = []; % the final indices of ''the members of set_out'' in set_in

candidate_ele = set_in; % initially, all pairs are candidates.
ia = (1:N)';

tol = 10^-4;
for i = 1:N % eliminate small errors 
    if abs(candidate_ele(i,1))<tol
        candidate_ele(i,1) = 0;
    end
    if abs(candidate_ele(i,2))<tol
        candidate_ele(i,2) = 0;
    end
end
sign_ele = sign(candidate_ele(:,1))+sign(candidate_ele(:,2)); % sum of signs
largest_sign_sum = max(sign_ele); % find the maximum sign sum

largest_candidate_ele = candidate_ele(sign_ele==largest_sign_sum,:);% only largest-sign-sum pairs are possible
largest_ia = ia(sign_ele==largest_sign_sum,:); % the indices of the largest-sign-sum pairs

select_idx = true(1,size(largest_candidate_ele,1)); % assume all remaining candidates are picked
% check dominate first, then check dominate_or_equal
for ii_temp = 1:size(largest_candidate_ele,1)
    if select_idx(ii_temp) == true
        for jj_temp = 1:size(largest_candidate_ele,1) % find(select_idx==true)
            if is_dominated_by(largest_candidate_ele(ii_temp,:), largest_candidate_ele(jj_temp,:))
                select_idx(ii_temp) = false;
                break
            end
        end
    end
    if select_idx(ii_temp) == true
        select_idx(ii_temp) = false;
        set_out = cat(1, set_out, largest_candidate_ele(ii_temp,:));
        set_idx = cat(1, set_idx, largest_ia(ii_temp));
    end
end



end

