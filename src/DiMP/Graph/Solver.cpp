#include <DiMP/Graph/Solver.h>

#include <sbtimer.h>
#include <sbrandom.h>

#include <algorithm>
#include <random>

namespace DiMP{;

static const real_t inf = numeric_limits<real_t>::max();
static Timer timer;
static Timer timer2;

///////////////////////////////////////////////////////////////////////////////////////////////////

DDPStage::DDPStage(CustomSolver* _solver){
    solver = _solver;
}

DDPStage::~DDPStage(){

}

void DDPStage::Init(){
    int nx = solver->state[0]->dim;
    int nu = solver->input[0]->dim;

    fcor.Allocate(nx);
    fx  .Allocate(nx, nx);
    fu  .Allocate(nx, nu);
}

void DDPStage::Prepare(){
    if(k < solver->N){
        vec_copy(solver->f_cor[k], fcor);
        mat_copy(solver->fx   [k], fx  );
        mat_copy(solver->fu   [k], fu  );
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////

DDPThread::DDPThread(CustomSolver* _solver){
    solver = _solver;
}

DDPThread::~DDPThread(){

}

void DDPThread::Init(){
    int N  = solver->N;
    int nx = solver->state[0]->dim;
    int nu = solver->input[0]->dim;
    
    path.resize(N+1);

    dx.resize(N+1);
    du.resize(N);

    Q         .resize(N);
	Qx        .resize(N);
	Qu        .resize(N);
	Qxx       .resize(N);
	Quu       .resize(N);
	Qux       .resize(N);
	Quuinv    .resize(N);
	Quuinv_Qu .resize(N);
    Quuinv_Qux.resize(N);
	V         .resize(N+1);
	Vx        .resize(N+1);
	Vxx       .resize(N+1);
    Vxxinv    .resize(N+1);
    
    Vxx_fcor        .resize(N);
    Vxx_fx          .resize(N);
    Vxx_fu          .resize(N);
    Vx_plus_Vxx_fcor.resize(N);
    Qu_plus_Qux_dx  .resize(N);

    for(int k = 0; k <= N; k++){
        if(k < N){
            du              [k].Allocate(nu);
            Qx              [k].Allocate(nx);
	        Qu              [k].Allocate(nu);
	        Qxx             [k].Allocate(nx, nx);
	        Quu             [k].Allocate(nu, nu);
	        Qux             [k].Allocate(nu, nx);
	        Quuinv          [k].Allocate(nu, nu);
	        Quuinv_Qu       [k].Allocate(nu);
            Quuinv_Qux      [k].Allocate(nu, nx);
            Vxx_fcor        [k].Allocate(nx);
            Vxx_fx          [k].Allocate(nx, nx);
            Vxx_fu          [k].Allocate(nx, nu);
            Vx_plus_Vxx_fcor[k].Allocate(nx);
            Qu_plus_Qux_dx  [k].Allocate(nu);

        }
        dx    [k].Allocate(nx);
	    Vx    [k].Allocate(nx);
	    Vxx   [k].Allocate(nx, nx);
        Vxxinv[k].Allocate(nx, nx);
    }
}

void DDPThread::Backward(){
    int N = solver->N;

    V[N] = path[N]->L;
    vec_copy(path[N]->Lx , Vx [N]);
    mat_copy(path[N]->Lxx, Vxx[N]);

    for(int k = N-1; k >= 0; k--){
        timer.CountUS();
        mat_vec_mul(Vxx[k+1], solver->stages[k]->fcor, Vxx_fcor[k], 1.0, 0.0);
        mat_mat_mul(Vxx[k+1], solver->stages[k]->fx  , Vxx_fx  [k], 1.0, 0.0);
        mat_mat_mul(Vxx[k+1], solver->stages[k]->fu  , Vxx_fu  [k], 1.0, 0.0);
        vec_copy(Vx[k+1]    , Vx_plus_Vxx_fcor[k]);
        vec_add (Vxx_fcor[k], Vx_plus_Vxx_fcor[k]);

        Q[k] = path[k]->L + V[k+1] + vec_dot(Vx[k+1], solver->stages[k]->fcor) + (1.0/2.0)*vec_dot(solver->stages[k]->fcor, Vxx_fcor[k]);
        
        vec_copy(path[k]->Lx, Qx[k]);
        mattr_vec_mul(solver->stages[k]->fx, Vx_plus_Vxx_fcor[k], Qx[k], 1.0, 1.0);

        vec_copy(path[k]->Lu, Qu[k]);
        mattr_vec_mul(solver->stages[k]->fu, Vx_plus_Vxx_fcor[k], Qu[k], 1.0, 1.0);

        mat_copy(path[k]->Lxx, Qxx[k]);
        mattr_mat_mul(solver->stages[k]->fx, Vxx_fx[k], Qxx[k], 1.0, 1.0);

        mat_copy(path[k]->Luu, Quu[k]);
        mattr_mat_mul(solver->stages[k]->fu, Vxx_fu[k], Quu[k], 1.0, 1.0);

        mat_copy(path[k]->Lux, Qux[k]);
        mattr_mat_mul(solver->stages[k]->fu, Vxx_fx[k], Qux[k], 1.0, 1.0);

        //Q   = state->L   + np->V + np->Vx*solver->f_cor[k] + (1.0/2.0)*(solver->f_cor[k]*(np->Vxx*solver->f_cor[k]));
        //Qx  = state->Lx  + solver->fx[k].trans()*(np->Vx + np->Vxx*solver->f_cor[k]);
        //Qu  = state->Lu  + solver->fu[k].trans()*(np->Vx + np->Vxx*solver->f_cor[k]);
        //Qxx = state->Lxx + solver->fx[k].trans()*(np->Vxx)*solver->fx[k];
        //Quu = state->Luu + solver->fu[k].trans()*(np->Vxx)*solver->fu[k];
        //Qux = state->Lux + solver->fu[k].trans()*(np->Vxx)*solver->fx[k];

	    for(int i = 0; i < Quu[k].m; i++)
		    Quu[k](i,i) += solver->param.regularization;
    	int T1 = timer.CountUS();

        timer.CountUS();
	    mat_inv_pd(Quu[k], Quuinv[k]);
        int T2 = timer.CountUS();

        timer.CountUS();
        symmat_vec_mul(Quuinv[k], Qu[k], Quuinv_Qu[k], 1.0, 0.0);
	    //Quuinv_Qu = Quuinv*Qu;

        V[k] = Q[k] - (1.0/2.0)*vec_dot(Qu[k], Quuinv_Qu[k]);
        
        vec_copy(Qx[k], Vx[k]);
        mattr_vec_mul(Qux[k], Quuinv_Qu[k], Vx[k], -1.0, 1.0);
	    
        mat_copy(Qxx[k], Vxx[k]);
        symmat_mat_mul(Quuinv[k], Qux[k], Quuinv_Qux[k], 1.0, 0.0);
        mattr_mat_mul(Qux[k], Quuinv_Qux[k], Vxx[k], -1.0, 1.0);
        //V   = Q   - (1.0/2.0)*(Qu*Quuinv_Qu);
	    //Vx  = Qx  - Qux.trans()*Quuinv_Qu ;
	    //Vxx = Qxx - Qux.trans()*Quuinv*Qux;
		
        // enforce symmetry of Uxx
	    for(int i = 1; i < Vxx[k].m; i++) for(int j = 0; j < i; j++)
		    Vxx[k](i,j) = Vxx[k](j,i);
        int T3 = timer.CountUS();

        //DSTR << T1 << " " << T2 << " " << T3 << endl;
    }
}

void DDPThread::Forward(){
    int N = solver->N;

    mat_inv_pd(Vxx[0], Vxxinv[0]);
    symmat_vec_mul(Vxxinv[0], Vx[0], dx[0], -1.0, 0.0);
    //vec_clear(dx);

    for(int k = 0; k < N; k++){
        vec_copy(Qu[k], Qu_plus_Qux_dx[k]);
        mat_vec_mul(Qux[k], dx[k], Qu_plus_Qux_dx[k], 1.0, 1.0);
        symmat_vec_mul(Quuinv[k], Qu_plus_Qux_dx[k], du[k], -1.0, 0.0);

        vec_copy   (solver->stages[k]->fcor, dx[k+1]);
        mat_vec_mul(solver->stages[k]->fx  , dx[k], dx[k+1], 1.0, 1.0);
        mat_vec_mul(solver->stages[k]->fu  , du[k], dx[k+1], 1.0, 1.0);

        //du[k  ] = -n->Quuinv*(n->Qu + n->Qux*dx[k]);
        //dx[k+1] = fx[k]*dx[k] + fu[k]*du[k] + f_cor[k];
    }
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////

DDPState::DDPState(CustomSolver* _solver){
    solver = _solver;
}

DDPState::~DDPState(){

}

void DDPState::Init(){
    int nx = solver->state[0]->dim;
    int nu = solver->input[0]->dim;

    Lx .resize(nx);
    Lu .resize(nu);
    Lxx.resize(nx, nx);
    Luu.resize(nu, nu);
    Lux.resize(nu, nx);

    _Lx    .Allocate(nx);
    _Lu    .Allocate(nu);
    _Lxx   .Allocate(nx, nx);
    _Luu   .Allocate(nu, nu);
    _Lux   .Allocate(nu, nx);
    //_Lxx_dx.Allocate(nx);
    //_Luu_du.Allocate(nu);
    //_Lux_dx.Allocate(nu);
}

void DDPState::CalcCost(){
    vec_copy(Lx , _Lx );
    vec_copy(Lu , _Lu );
    mat_copy(Lxx, _Lxx);
    mat_copy(Luu, _Luu);
    mat_copy(Lux, _Lux);

    //mat_vec_mul(_Lxx, stage->dx, _Lxx_dx, 1.0, 0.0);
    //mat_vec_mul(_Luu, stage->du, _Luu_du, 1.0, 0.0);
    //mat_vec_mul(_Lux, stage->dx, _Lux_dx, 1.0, 0.0);

    //J = L + vec_dot(_Lx, stage->dx) + vec_dot(_Lu, stage->du)
    //    + (1.0/2.0)*vec_dot(stage->dx, _Lxx_dx)
    //    + (1.0/2.0)*vec_dot(stage->du, _Luu_du)
    //    +           vec_dot(stage->du, _Lux_dx);
}

void DDPState::Finish(){

}

void DDPState::Print(){

}
	
///////////////////////////////////////////////////////////////////////////////////////////////////

void CustomSolver::Init(){
    Solver::Init();

    if(param.methodMajor == CustomMethod::SearchDDP){
        InitDDP();

        stages.resize(N+1);
        for(int k = 0; k <= N; k++){
            stages[k] = new DDPStage(this);
        }
        for(int k = 0; k <= N; k++){
            stages[k]->k = k;
            stages[k]->next = (k < N ? stages[k+1] : (DDPStage*)0);
            stages[k]->Init();
        }

        DDPState* st = callback->CreateInitialState();
        st->Init();
        st->stage = stages[0];
        stages[0]->states.push_back(st);
            
        for(int k = 0; k < N; k++){
            for(DDPState* st0 : stages[k]->states){
                vector<DDPState*> _next;
                callback->CreateNextStates(st0, _next);
                
                DDPState* st1;
                for(DDPState* _st1 : _next){
                    st1 = 0;
                    for(DDPState* __st1 : stages[k+1]->states){
                        if(__st1->IsIdentical(_st1)){
                            st1 = __st1;
                            break;
                        }
                    }
                    if(!st1){
                        stages[k+1]->states.push_back(_st1);
                        st1 = _st1;
                    }
                    st1->stage = stages[k+1];
                    st1->Init();
                    st0->next.insert(st1);
                    st1->prev.insert(st0);
                }
            }
        }

    }
}

void CustomSolver::CalcDirection(){
    if(param.methodMajor == CustomMethod::SearchDDP){
        timer2.CountUS();
		CalcDirectionSearchDDP();
        int timeDir = timer2.CountUS();
        DSTR << "time dir: " << timeDir << endl;
    }
    else{
        Solver::CalcDirection();
    }
}

//void CustomSolver::Shuffle(const vector<DDPState*>& path0, vector<DDPState*>& path1){
void CustomSolver::Shuffle(){
    vector< vector<DDPState*> > st_int;
    st_int.resize(N);
    vector< pair<int, DDPState*> >  idx;
    
    const vector<DDPState*>& path0 = threads[0]->path;
    for(int k = 1; k <= N-1; k++){
        st_int[k].resize(std::min(path0[k-1]->next.size(), path0[k+1]->prev.size()));
        vector<DDPState*>::iterator it = set_intersection(
            path0[k-1]->next.begin(), path0[k-1]->next.end(),
            path0[k+1]->prev.begin(), path0[k+1]->prev.end(),
            st_int[k].begin());
        st_int[k].resize(it - st_int[k].begin());
        st_int[k].erase(find(st_int[k].begin(), st_int[k].end(), path0[k]));

        for(DDPState* st : st_int[k])
            idx.push_back(make_pair(k, st));

    }

    static std::default_random_engine urng(0);
    shuffle(idx.begin(), idx.end(), urng);
    
    for(int i = 1; i < threads.size(); i++){
        threads[i]->path = threads[0]->path;
        int j = (i-1)%idx.size();
        threads[i]->path[idx[j].first] = idx[j].second;
    }
    /*
    static Sampler sampler;
    vector<DDPState*> st_int;
    path1 = path0;
    while(true){
        int k = sampler.SampleInt(1, N-1);
        st_int.resize(std::min(path0[k-1]->next.size(), path0[k+1]->prev.size()));
        vector<DDPState*>::iterator it = set_intersection(
            path0[k-1]->next.begin(), path0[k-1]->next.end(),
            path0[k+1]->prev.begin(), path0[k+1]->prev.end(),
            st_int.begin());
        st_int.resize(it - st_int.begin());
        st_int.erase(find(st_int.begin(), st_int.end(), path0[k]));

        if(st_int.size() == 0)
            continue;

        path1[k] = st_int[sampler.SampleInt(0, st_int.size()-1)];
        break;
    }
    */
}    

void CustomSolver::CompDP(vector<DDPState*>& path){
    for(DDPState* st : stages[N]->states){
        st->Jopt    = (st->IsTerminal() ? st->L : 10000);
        st->nextOpt = 0;
    }

    for(int k = N-1; k >= 0; k--){
        for(DDPState* st0 : stages[k]->states){
            st0->nextOpt = 0;
            for(DDPState* st1 : st0->next){
                if(!st0->nextOpt || st1->Jopt < st0->nextOpt->Jopt){
                    st0->nextOpt = st1;
                }
            }
            st0->Jopt = st0->L + st0->nextOpt->Jopt;
        }
    }

    path.clear();
    DDPState* st = stages[0]->states[0];
    while(st){
        path.push_back(st);
        st = st->nextOpt;
    }

}

void CustomSolver::CalcDirectionSearchDDP(){
    timer.CountUS();
    for(auto& con : cons_active)
		con->CalcCorrection();

    PrepareDDP ();
    int timePrepare = timer.CountUS();

    int nx = state[0]->dim;
    int nu = input[0]->dim;

    // calc cost
    for(int k = 0; k <= N; k++){
        stages[k]->Prepare();

        for(DDPState* st : stages[k]->states){
            st->CalcCost();
        }
    }

    const int numSample = 50;

    // for the first time, compute initial guess of mode sequence
    if(threads.empty()){
        threads.resize(numSample);
        for(int i = 0; i < numSample; i++){
            threads[i] = new DDPThread(this);
            threads[i]->Init();
        }

        CompDP(threads[0]->path);
    }
    
    while(true){
        // create shuffled sequences
        Shuffle();
        //for(int i = 1; i < numSample; i++){
        //    Shuffle(threads[0]->path, threads[i]->path);
        //}

//#pragma omp parallel for  num_threads(20)
        for(int i = 0; i < numSample; i++){
            // perform DDP with previous mode sequence
            threads[i]->Backward();
        }

        int    iopt = -1;
        real_t Jopt;
        for(int i = 0; i < numSample; i++){
            if(iopt == -1 || threads[i]->V[0] < Jopt){
                iopt = i;
                Jopt = threads[i]->V[0];
            }
        }

        for(int k = 0; k <= N; k++)
            threads[iopt]->path[k]->Print();
        DSTR << Jopt << endl;
        
        if(iopt == 0)
            break;

        threads[0]->path = threads[iopt]->path;
    }

    threads[0]->Forward();

    for(int k = 0; k <= N; k++)
        threads[0]->path[k]->Finish();

	for(int k = 0; k <= N; k++){
		for(SubState* subst : state[k]->substate){
			if(subst->var->locked)
				continue;

			int j0 = subst->index;
			for(int j = 0; j < subst->var->nelem; j++){
				subst->var->dx[j] = threads[0]->dx[k](j0+j);
			}
		}
	}
	for(int k = 0; k < N; k++){
		for(SubInput* subin : input[k]->subinput){
			if(subin->var->locked)
				continue;

			int j0 = subin->index;
			for(int j = 0; j < subin->var->nelem; j++){
				subin->var->dx[j] = threads[0]->du[k](j0+j);
			}
		}
		
	}
}

}
