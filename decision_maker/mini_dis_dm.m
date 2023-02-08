function sel_idx = mini_dis_dm(opt_sol_set, a, b)
% a: \Trec
% b: H-\Tdur
assert(size(opt_sol_set,2)==2);

dis = zeros(size(opt_sol_set,1),1);

for i=1:length(dis)
    dis(i)= sqrt( (opt_sol_set(i,1)-a)^2 + (opt_sol_set(i,2)-b)^2 );
end

[~,sel_idx]  = min(dis);

end

