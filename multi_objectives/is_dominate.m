function a_dominate_b = is_dominate(a,b)
%IS_DOMINATE a pareto dominates b
assert(numel(a)==2);
assert(numel(b)==2);

tol = 10^-6;
a_dominate_b =  (a(1)-b(1)>tol && (a(2)>b(2)||abs(a(2)-b(2))<tol)) || ((a(1)>b(1)||abs(a(1)-b(1))<tol) && a(2)-b(2)>tol);

end

