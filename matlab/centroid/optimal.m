function [J, dx, du] = optimal(dx0, I)
	global nx
	global nu
	global N
    global fx
    global fu
    global f0

	dx  = cell(N  , 1);
	du  = cell(N-1, 1);
	fx  = cell(N-1, 1);
	fu  = cell(N-1, 1);
	f0  = cell(N-1, 1);
    L   = cell(N  , 1);
	Lx  = cell(N  , 1);
	Lu  = cell(N-1, 1);
	Lxx = cell(N  , 1);
	Luu = cell(N-1, 1);
	Lux = cell(N-1, 1);
    V   = cell(N  , 1);
    Vx  = cell(N  , 1);
    Vxx = cell(N  , 1);
    Q   = cell(N-1, 1);
    Qx  = cell(N-1, 1);
    Qu  = cell(N-1, 1);
    Qxx = cell(N-1, 1);
    Quu = cell(N-1, 1);
    Qux = cell(N-1, 1);
    Quuinv    = cell(N-1, 1);
    Quuinv_Qu = cell(N-1, 1);
    
    for k = 1:N
        dx {k} = zeros(nx, 1);
        L  {k} = 0;
        Lx {k} = zeros(nx, 1);
        Lxx{k} = zeros(nx, nx);
        V  {k} = 0;
        Vx {k} = zeros(nx, 1);
        Vxx{k} = zeros(nx, nx);

        if k < N
            du {k} = zeros(nu, 1);
            fx {k} = zeros(nx, nx);
            fu {k} = zeros(nx, nu);
            f0 {k} = zeros(nx, 1);
            Lu {k} = zeros(nu, 1);
            Luu{k} = zeros(nu, nu);
            Lux{k} = zeros(nu, nx);
            Qx {k} = zeros(nx, 1);
            Qu {k} = zeros(nu, 1);
            Qxx{k} = zeros(nx, nx);
            Quu{k} = zeros(nu, nx);
            Qux{k} = zeros(nu, nx);
            Quuinv{k}    = zeros(nu, nu);
            Quuinv_Qu{k} = zeros(nu, 1);
        end
    end
    
    includes_flight = 0;
    for k = 1:N-1
        if I(:,k) == [0;0]
            includes_flight = 1;
            break;
        end
        [fx{k}, fu{k}, f0{k}] = calc_matrices(I(:,k));
    end
    
    if includes_flight == 1
        J = inf;
        return;
    end
    
    for k = 1:N
        if k < N
            [L{k}, Lx{k}, Lxx{k}, Lu{k}, Luu{k}, Lux{k}] = calc_cost(k);
        else
            [L{k}, Lx{k}, Lxx{k}] = calc_cost(k);
        end
    end
    
    V  {N} = L  {N};
	Vx {N} = Lx {N};
	Vxx{N} = Lxx{N};
    
	for k = N-1:-1:1
        Q  {k} = L  {k} + V{k+1} + Vx{k+1}'*f0{k} + (1.0/2.0)*(f0{k}'*Vxx{k+1}*f0{k});
    	Qx {k} = Lx {k} + fx{k}'*(Vx{k+1} + Vxx{k+1}*f0{k});
    	Qu {k} = Lu {k} + fu{k}'*(Vx{k+1} + Vxx{k+1}*f0{k});
    	Qxx{k} = Lxx{k} + fx{k}'*Vxx{k+1}*fx{k};
    	Quu{k} = Luu{k} + fu{k}'*Vxx{k+1}*fu{k};
    	Qux{k} = Lux{k} + fu{k}'*Vxx{k+1}*fx{k};

		Quu{k} = Quu{k} + eps*eye(nu);
    	
        Quuinv{k} = Quu{k}^-1;
        
        % force symmetry
        Quuinv{k} = (1.0/2.0)*(Quuinv{k} + Quuinv{k}');

	    Quuinv_Qu{k} = Quuinv{k}*Qu{k};

	    V  {k} = Q  {k} - (1.0/2.0)*(Qu{k}'*Quuinv_Qu{k});
	    Vx {k} = Qx {k} - Qux{k}'*Quuinv_Qu{k};
	    Vxx{k} = Qxx{k} - Qux{k}'*Quuinv{k}*Qux{k};
            end
	
    dx{1} = dx0;
	for k = 1:N-1
	    du{k}   = -Quuinv{k}*(Qu{k} + Qux{k}*dx{k});
		dx{k+1} = fx{k}*dx{k} + fu{k}*du{k} + f0{k};
    end

    J = V{1} + Vx{1}'*dx0 + (1.0/2.0)*(dx0'*Vxx{1}*dx0);
end
