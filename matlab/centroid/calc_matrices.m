function [A, B, a] = calc_matrices(I)
	global nend
	global nx
	global nu
	global tau
	global lambda
	global g
	
	A = zeros(nx, nx);
	B = zeros(nx, nu);
	a = zeros(nx, 1);
	
	nc = sum(I);
	
	if nc == 0
		A(1:2, 1:2) =  eye(2);
		A(1:2, 3:4) =  eye(2)*tau;

		A(3:4, 3:4) = eye(2);
		
		for iend = 1:nend
			ix = 4+2*(iend-1);
			iu = 2*(iend-1);
			A(ix+1:ix+2, ix+1:ix+2) = eye(2);
			B(ix+1:ix+2, iu+1:iu+2) = eye(2)*tau;
		end		
	else
		C = cosh(lambda*tau);
		S = sinh(lambda*tau);
		
		A(1:2, 1:2) = eye(2)*C;
		A(1:2, 3:4) = eye(2)*(S/lambda);
		for iend = 1:nend
			ix = 4+2*(iend-1);
			A(1:2, ix+1:ix+2) = eye(2)*((1-C)/nc);
		end
		
		A(3:4, 1:2) = eye(2)*(lambda*S);
		A(3:4, 3:4) = eye(2)*C;
		for iend = 1:nend
			ix = 4+2*(iend-1);
			A(1:2, ix+1:ix+2) = eye(2)*((-lambda*S)/nc);
		end
		
		for iend = 1:nend
			ix = 4+2*(iend-1);
			iu = 2*(iend-1);
			A(ix+1:ix+2, ix+1:ix+2) = eye(2);
			B(ix+1:ix+2, iu+1:iu+2) = eye(2)*((1 - I(iend))*tau);
		end
	end
end
