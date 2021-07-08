function [J, U] = optimal(x0, I)
	global nx
	global nu
	global N

	A = zeros(N-1, nx, nx);
	B = zeros(N-1, nx, nu);
	a = zeros(N-1, nx, 1);
	
	for k = 1:N-1
		[A(k), B(k), a(k)] = calc_matrices
	end
end
