function rec_dur_pair = compute_pair_over_traj(bool_trace, alpha, beta)

trajLength = length(bool_trace);

tol = 0.002;
first_true = find(abs(bool_trace-1)<tol,1);

if isempty(first_true)
    recoverability = trajLength-1 - 0;
    t_rec = trajLength;
else
    recoverability = first_true-1 - 0;
    t_rec = first_true;
end
first_false = find(abs(bool_trace(t_rec:end))<tol, 1);
if isempty(first_false)
    durability = trajLength - t_rec+1;
else
    durability = first_false -1 - 0;
end

rec_dur_pair = [alpha-recoverability; durability-beta];

end