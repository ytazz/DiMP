%
% centroid test code
%
global nend
global nx
global nu
global N
global tau
global lambda

% num of ends
nend = 2;

% state dimension
nx = 3+3+2*nend;

% input dimension
nu = 2*nend;

% num of steps
N = 10;

% duration
tau = 0.3;

% stiffness
lambda = 1.0;

% initial state
% px py pz vx vy vz p1x p1y p2x p2y
x0 = [0.0
      0.0
      1.0
      0.0
      0.0
      0.0
      0.0
     -0.1
      0.0
      0.1];
      
% initial contact state
I0 = [1
      1];
      
% initial mode sequence
I    = repmat(I0, 1, N);

% set of neighboring mode sequences
nneighbor = nend*(N-1)+1;
Iset = zeros(nneighbor*nend, N);

% optimal control input sequence
U = zeros(nneighbor*nu, N-1);

% minimum cost
J = zeros(nneighbor, 1);

while
	% generate neighboring mode sequences
	Iset = repmat(I, nneighbor, 1);
	ofst = nend;
	for iend = 1:nend
		for k = 2:N
			Iset(ofst + iend, k) = !Iset(ofst + iend, k);
			ofst = ofst + nend;
		end
	end
	
	% for each mode sequence
	for i = 1:nneighbor
		% calc optimal control
		[J(i), U((i-1)*nu+1:i*nu,:)] = optimal(x0, Iset((i-1)*nend+1:i*nend,:));
		
	end
	
	% update best mode sequence
	
	% break if mode sequence has not changed (local optimality reached)

end

