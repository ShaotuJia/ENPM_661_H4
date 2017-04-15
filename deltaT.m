%This function is to find delta_t in epsilon 0.25

function delta_t = deltaT(v_init, a, epsilon)

syms t;
delta_epsilon = @(t) v_init * t + (1/2) * a * (t^2) - epsilon;
roots = solve(delta_epsilon,t, 'Real', true);
roots = double(roots);
roots = roots(roots > 0);
delta_t = min(roots);

end