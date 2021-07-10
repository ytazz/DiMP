function [J, dx, du] = optimal(dx0, I)
	global nx
	global nu
	global N

	dx  = zeros(N,   nx, 1);
	du  = zeros(N-1, nu, 1);
	fx  = zeros(N-1, nx, nx);
	fu  = zeros(N-1, nx, nu);
	f0  = zeros(N-1, nx, 1);
	Lx  = zeros(N-1, nx, 1);
	Lu  = zeros(N-1, nu, 1);
	Lxx = zeros(N-1, nx, nx);
	Luu = zeros(N-1, nu, nu);
	Lux = zeros(N-1, nu, nx);
    
	for k = 1:N-1
		[A, B, a] = calc_matrices(I(:,k));
        fx(k,:,:) = A;
        fu(k,:,:) = B;
        f0(k,:,:) = a;
        %size(fx(k,:,:))
	end
	for k = 1:N
		[L(k), Lx(k), Lu(k), Lxx(k), Luu(k), Lux(k)] = calc_cost(k);
	end
	
	V(N)   = L(N);
	Vx(N)  = Lx(N);
	Vxx(N) = Lxx(N);

	for k = N-1:-1:1
        Q(k)   = L(k)   + V(k+1) + Vx(k+1)*f0(k) + (1.0/2.0)*(f0(k)'*Vxx(k+1)*f0(k));
    	Qx(k)  = Lx(k)  + fx(k)'*(Vx(k+1) + Vxx(k+1)*f0(k));
    	Qu(k)  = Lu(k)  + fu(k)'*(Vx(k+1) + Vxx(k+1)*f0(k));
    	Qxx(k) = Lxx(k) + fx(k)'*Vxx(k+1)*fx(k);
    	Quu(k) = Luu(k) + fu(k)'*Vxx(k+1)*fu(k);
    	Qux(k) = Lux(k) + fu(k)'*Vxx(k+1)*fx(k);

		Quu(k) = Quu(k) + eps*eye(nu);
    	
        Quuinv(k) = Quu(k)^-1;

	    Quuinv_Qu(k) = Quuinv(k)*Qu(k);

	    V(k)   = Q(k)   - (1.0/2.0)*(Qu(k)'*Quuinv_Qu(k));
	    Vx(k)  = Qx(k)  - Qux(k)'*Quuinv_Qu(k);
	    Vxx(k) = Qxx(k) - Qux(k)'*Quuinv(k)*Qux(k);
	end
	
	for k = 1:N
	    du(k)   = -Quuinv(k)*(Qu(k) + Qux(k)*dx(k));
		dx(k+1) = fx(k)*dx(k) + fu(k)*du(k) + f0(k);
	end
	
	J = V(1);
end
