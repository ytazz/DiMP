%
% centroid test code
%
global nend
global nx
global nu
global N
global tau
global lambda
global g
global wend
global wvel
global wdes
global pgoal

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

% gravity
g = [0;0;1];

% cost weights
wend = 1.0;
wvel = 1.0;
wdes = 10.0;

% goal position
pgoal = [1;
         0;
         1];

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
Iset = cell(nneighbor, 1);

% optimal control input sequence
U = cell(nneighbor, 1);

% optimal state sequence
X = cell(nneighbor, 1);

% minimum cost
J = cell(nneighbor, 1);

while 1
	% generate neighboring mode sequences
    for i = 1:nneighbor
        Iset{i} = I;
    end
    
	i = 1;
	for iend = 1:nend
		for k = 2:N
            % logical not
			Iset{i}(iend, k) = ~Iset{i}(iend, k);
			i = i+1;
		end
	end
	
	% for each mode sequence
	for i = 1:nneighbor
		% calc optimal control
		[J{i}, U{i}, X{i}] = optimal(x0, Iset{i});
		
    end
	
    'hoge'
    
	% update best mode sequence
	
	% break if mode sequence has not changed (local optimality reached)

end

