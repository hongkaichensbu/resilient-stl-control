function a_b_do_not_dominate_each_other = is_equal(a,b)
%IS_EQUAL a does not dominate b and vice versa
a_b_do_not_dominate_each_other = (~is_dominate(a,b)) && (~is_dominated_by(a,b));
end

