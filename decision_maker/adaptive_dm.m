function sel_idx = adaptive_dm(opt_sol_set, ~, ~)

max_rec = max(opt_sol_set(:,1));
max_dur = max(opt_sol_set(:,2));

if max_rec < max_dur 
sel_idx = pro_rec_dm(opt_sol_set);
elseif max_rec > max_dur
sel_idx = pro_dur_dm(opt_sol_set);
else
sel_idx = random_dm(opt_sol_set);
end

end
