function sel_idx = pro_rec_dm(opt_sol_set, ~, ~)

assert(size(opt_sol_set,2)==2);

[~,sel_idx] = max(opt_sol_set(:,1));

end

