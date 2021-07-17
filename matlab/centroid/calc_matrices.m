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
		A(1:3, 1:3) =  eye(3);
		A(1:3, 4:6) =  eye(3)*tau;
		a(1:3)      = -0.5*g*tau^2;

		A(4:6, 4:6) = eye(3);
		a(4:6)      = -g*tau;
		
		for iend = 1:nend
			ix = 6+2*(iend-1);
			iu = 2*(iend-1);
			A(ix+1:ix+2, ix+1:ix+2) = eye(2);
			B(ix+1:ix+2, iu+1:iu+2) = eye(2)*tau;
		end		
	else
		C = cosh(lambda*tau);
		S = sinh(lambda*tau);
		
		A(1:3, 1:3) = eye(3)*C;
		A(1:3, 4:6) = eye(3)*(S/lambda);
		for iend = 1:nend
			ix = 6+2*(iend-1);
			A(1:3, ix+1:ix+2) = [eye(2); zeros(1,2)]*((1-C)/nc);
		end
		a(1:3) = ((1-C)/(lambda^2))*g;
		
		A(4:6, 1:3) = eye(3)*(lambda*S);
		A(4:6, 4:6) = eye(3)*C;
		for iend = 1:nend
			ix = 6+2*(iend-1);
			A(1:3, ix+1:ix+2) = [eye(2); zeros(1,2)]*((-lambda*S)/nc);
		end
		a(1:3) = ((-S)/(lambda))*g;

		for iend = 1:nend
			ix = 6+2*(iend-1);
			iu = 2*(iend-1);
			A(ix+1:ix+2, ix+1:ix+2) = eye(2);
			B(ix+1:ix+2, iu+1:iu+2) = eye(2)*((1 - I(iend))*tau);
		end
	end
end
