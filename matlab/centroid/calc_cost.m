function [L, Lx, Lxx, Lu, Luu, Lux] = calc_cost(k)
	global nend
	global nx
	global nu
	global N
	global pgoal
	global wend
	global wvel
	global wdes
	
	L   = 0;
	Lx  = zeros(nx, 1);
	Lu  = zeros(nu, 1);
	Lxx = zeros(nx, nx);
	Luu = zeros(nu, nu);
	Lux = zeros(nu, nx);
	
	% end effector range
	% 1/2*wend*||p - pend||^2
	for iend = 1:nend
		ix = 4 + 2*(iend - 1);
        Lxx(1:2, 1:2) = Lxx(1:2, 1:2) + wend*eye(2);
		Lxx(1:2, ix+1:ix+2) = Lxx(1:2, ix+1:ix+2) - wend*eye(2);
		Lxx(ix+1:ix+2, 1:2) = Lxx(ix+1:ix+2, 1:2) - wend*eye(2);
		Lxx(ix+1:ix+2, ix+1:ix+2) = Lxx(ix+1:ix+2, ix+1:ix+2) + wend*eye(2);
	end

	% end effector velocity
	% 1/2*wvel*||vend||^2
	for iend = 1:nend
		iu = 2*(iend - 1);
		Luu(iu+1:iu+2, iu+1:iu+2) = wvel*eye(2);
	end
	
	% desired position
	% 1/2*wdes*||p - pgoal||^2
	if k == N
		L = L + 0.5*wdes*(pgoal'*pgoal);
		Lx(1:2, 1) = Lx(1:2, 1) - wdes*pgoal;
		Lxx(1:2, 1:2) = Lxx(1:2, 1:2) + wdes*eye(2);
	end

end
