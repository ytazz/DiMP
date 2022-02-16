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

///////////////////////////////////////////////////////////////////////////////////////////////////

DDPStep::DDPStep(DDPThread* _thread){
    thread = _thread;
}

void DDPStep::Init(){
    int N  = thread->solver->N;
    int nx = thread->solver->state[0]->dim;
    int nu = thread->solver->input[0]->dim;

    if(k < N){
        du              .Allocate(nu);
        Qx              .Allocate(nx);
	    Qu              .Allocate(nu);
	    Qxx             .Allocate(nx, nx);
	    Quu             .Allocate(nu, nu);
	    Qux             .Allocate(nu, nx);
	    Quuinv          .Allocate(nu, nu);
	    Quuinv_Qu       .Allocate(nu);
        Quuinv_Qux      .Allocate(nu, nx);
        Vxx_fcor        .Allocate(nx);
        Vxx_fx          .Allocate(nx, nx);
        Vxx_fu          .Allocate(nx, nu);
        Vx_plus_Vxx_fcor.Allocate(nx);
        Qu_plus_Qux_dx  .Allocate(nu);

        Px        .Allocate(nx);
	    Pu        .Allocate(nu);
	    Pxx       .Allocate(nx, nx);
	    Puu       .Allocate(nu, nu);
	    Pux       .Allocate(nu, nx);
	    Puuinv    .Allocate(nu, nu);
	    Puuinv_Pu .Allocate(nu);
        Puuinv_Pux.Allocate(nu, nx);
        Lx_plus_Ux                           .Allocate(nx);
        Lxx_plus_Uxx                         .Allocate(nx, nx);
        Lxx_plus_Uxx_fcor_rev                .Allocate(nx);
        Lx_plus_Ux_plus_Lxx_plus_Uxx_fcor_rev.Allocate(nx);
        Lxx_plus_Uxx_fx_rev                  .Allocate(nx, nx);
        Lxx_plus_Uxx_fu_rev                  .Allocate(nx, nu);
        Lux_fx_rev                           .Allocate(nu, nx);
    }
    dx       .Allocate(nx);
	Vx       .Allocate(nx);
	Vxx      .Allocate(nx, nx);
    Vxxinv   .Allocate(nx, nx);
    Ux       .Allocate(nx);
	Uxx      .Allocate(nx, nx);
    Uxxinv   .Allocate(nx, nx);
    Ux_plus_Vx                 .Allocate(nx);
    Uxx_plus_Vxx               .Allocate(nx, nx);
    Uxx_plus_Vxx_inv           .Allocate(nx, nx);
    Uxx_plus_Vxx_inv_Ux_plus_Vx.Allocate(nx);
}

void DDPStep::CalcValueForward(){
    if(!prev){
        U = 0.0;
        vec_clear(Ux );
        mat_clear(Uxx);
    }
    if(next){
        vec_copy   (state->Lx , Lx_plus_Ux  );
        vec_add    (Ux        , Lx_plus_Ux  );
        mat_copy   (state->Lxx, Lxx_plus_Uxx);
        mat_add    (Uxx       , Lxx_plus_Uxx);
        mat_vec_mul(Lxx_plus_Uxx, thread->fcor_rev[k], Lxx_plus_Uxx_fcor_rev, 1.0, 0.0);
        vec_copy   (Lx_plus_Ux           , Lx_plus_Ux_plus_Lxx_plus_Uxx_fcor_rev);
        vec_add    (Lxx_plus_Uxx_fcor_rev, Lx_plus_Ux_plus_Lxx_plus_Uxx_fcor_rev);
        mat_mat_mul(Lxx_plus_Uxx, thread->fx_rev[k], Lxx_plus_Uxx_fx_rev, 1.0, 0.0);
        mat_mat_mul(Lxx_plus_Uxx, thread->fu_rev[k], Lxx_plus_Uxx_fu_rev, 1.0, 0.0);
        mat_mat_mul(state->Lux  , thread->fx_rev[k], Lux_fx_rev, 1.0, 0.0);

        P = state->L + U + vec_dot(Lx_plus_Ux, thread->fcor_rev[k]) + (1.0/2.0)*vec_dot(thread->fcor_rev[k], Lxx_plus_Uxx_fcor_rev);
        
        mattr_vec_mul(thread->fx_rev[k], Lx_plus_Ux_plus_Lxx_plus_Uxx_fcor_rev, Px, 1.0, 0.0);
        
        vec_copy(state->Lu, Pu);
        mattr_vec_mul(thread->fu_rev[k], Lx_plus_Ux_plus_Lxx_plus_Uxx_fcor_rev, Pu, 1.0, 1.0);
        
        mattr_mat_mul(thread->fx_rev[k], Lxx_plus_Uxx_fx_rev, Pxx, 1.0, 0.0);
        
        mat_copy(state->Luu, Puu);
        mattr_mat_mul(thread->fu_rev[k], Lxx_plus_Uxx_fu_rev, Puu, 1.0, 1.0);
        
        mat_copy(Lux_fx_rev, Pux);
        mattr_mat_mul(thread->fu_rev[k], Lxx_plus_Uxx_fx_rev, Pux, 1.0, 1.0);

        //Px[k] = fx_rev[k].trans()*(Lx [k] + Ux [k] + (Lxx[k] + Uxx[k])*fcor_rev[k]);
        //Pu[k] = fu_rev[k].trans()*(Lx [k] + Ux [k] + (Lxx[k] + Uxx[k])*fcor_rev[k]) + Lu[k];
        //Pxx[k] = fx_rev[k].trans()*(Lxx[k] + Uxx[k])*fx_rev[k];
        //Puu[k] = fu_rev[k].trans()*(Lxx[k] + Uxx[k])*fu_rev[k] + Luu[k];
        //Pux[k] = (fu_rev[k].trans()*(Lxx[k] + Uxx[k]) + Lux[k])*fx_rev[k];

        for(int i = 0; i < Puu.m; i++)
		    Puu(i,i) += thread->solver->param.regularization;
    	
	    mat_inv_pd(Puu, Puuinv);

        symmat_vec_mul(Puuinv, Pu , Puuinv_Pu , 1.0, 0.0);
	    symmat_mat_mul(Puuinv, Pux, Puuinv_Pux, 1.0, 0.0);
        
	    next->U = P - (1.0/2.0)*vec_dot(Pu, Puuinv_Pu);
	    
        vec_copy(Px, next->Ux);
        mattr_vec_mul(Pux, Puuinv_Pu, next->Ux, -1.0, 1.0);
	    
        mat_copy(Pxx, next->Uxx);
        mattr_mat_mul(Pux, Puuinv_Pux, next->Uxx, -1.0, 1.0);

        //Puuinv_Pu[k] = Puuinv[k]*Pu[k];
        //Ux [k+1] = Px [k] - Pux[k].trans()*Puuinv_Pu[k];
        //Uxx[k+1] = Pxx[k] - Pux[k].trans()*Puuinv[k]*Pux[k];
		
        // enforce symmetry of Uxx
	    for(int i = 1; i < next->Uxx.m; i++) for(int j = 0; j < i; j++)
		    next->Uxx(i,j) = next->Uxx(j,i);

    }

    // add terminal cost
    //U  [N] += L  [N];
    //Ux [N] += Lx [N];
    //Uxx[N] += Lxx[N];
    //mat_inv_sym(Uxx[N], Uxxinv);
}

void DDPStep::CalcValueBackward(){
    if(!next){
        V = state->L;
        vec_copy(state->Lx , Vx );
        mat_copy(state->Lxx, Vxx);
    }
    else{
        mat_vec_mul(next->Vxx, thread->fcor[k], Vxx_fcor, 1.0, 0.0);
        mat_mat_mul(next->Vxx, thread->fx  [k], Vxx_fx  , 1.0, 0.0);
        mat_mat_mul(next->Vxx, thread->fu  [k], Vxx_fu  , 1.0, 0.0);
        vec_copy(next->Vx, Vx_plus_Vxx_fcor);
        vec_add (Vxx_fcor, Vx_plus_Vxx_fcor);

        Q = state->L + next->V + vec_dot(next->Vx, thread->fcor[k]) + (1.0/2.0)*vec_dot(thread->fcor[k], Vxx_fcor);
        
        vec_copy(state->Lx, Qx);
        mattr_vec_mul(thread->fx[k], Vx_plus_Vxx_fcor, Qx, 1.0, 1.0);

        vec_copy(state->Lu, Qu);
        mattr_vec_mul(thread->fu[k], Vx_plus_Vxx_fcor, Qu, 1.0, 1.0);

        mat_copy(state->Lxx, Qxx);
        mattr_mat_mul(thread->fx[k], Vxx_fx, Qxx, 1.0, 1.0);

        mat_copy(state->Luu, Quu);
        mattr_mat_mul(thread->fu[k], Vxx_fu, Quu, 1.0, 1.0);

        mat_copy(state->Lux, Qux);
        mattr_mat_mul(thread->fu[k], Vxx_fx, Qux, 1.0, 1.0);

        for(int i = 0; i < Quu.m; i++)
		    Quu(i,i) += thread->solver->param.regularization;
    	
        mat_inv_pd(Quu, Quuinv);
        
        symmat_vec_mul(Quuinv, Qu , Quuinv_Qu , 1.0, 0.0);
	    symmat_mat_mul(Quuinv, Qux, Quuinv_Qux, 1.0, 0.0);
        
        V = Q - (1.0/2.0)*vec_dot(Qu, Quuinv_Qu);
        
        vec_copy(Qx, Vx);
        mattr_vec_mul(Qux, Quuinv_Qu, Vx, -1.0, 1.0);
	    
        mat_copy(Qxx, Vxx);
        mattr_mat_mul(Qux, Quuinv_Qux, Vxx, -1.0, 1.0);
        
        // enforce symmetry of Uxx
	    for(int i = 1; i < Vxx.m; i++) for(int j = 0; j < i; j++)
		    Vxx(i,j) = Vxx(j,i);
    }
}

void DDPStep::CalcStateForward(){
    if(!prev){
        mat_inv_pd(Vxx, Vxxinv);
        symmat_vec_mul(Vxxinv, Vx, dx, -1.0, 0.0);
        //vec_clear(dx);
    }

    if(next){
        vec_copy(Qu, Qu_plus_Qux_dx);
        mat_vec_mul(Qux, dx, Qu_plus_Qux_dx, 1.0, 1.0);
        symmat_vec_mul(Quuinv, Qu_plus_Qux_dx, du, -1.0, 0.0);

        vec_copy   (thread->fcor[k], next->dx);
        mat_vec_mul(thread->fx  [k], dx, next->dx, 1.0, 1.0);
        mat_vec_mul(thread->fu  [k], du, next->dx, 1.0, 1.0);

        //du[k  ] = -n->Quuinv*(n->Qu + n->Qux*dx[k]);
        //dx[k+1] = fx[k]*dx[k] + fu[k]*du[k] + f_cor[k];
    }   
}

void DDPStep::Apply(){
    thread->steps[k]->state = state;
}

void DDPStep::CalcCost(){
    CalcValueBackward();

    U_plus_V = thread->steps[k]->U + V;
    
    vec_copy(thread->steps[k]->Ux, Ux_plus_Vx);
    vec_add (Vx, Ux_plus_Vx);
    
    mat_copy(thread->steps[k]->Uxx, Uxx_plus_Vxx);
    mat_add (Vxx, Uxx_plus_Vxx);

    mat_inv_pd(Uxx_plus_Vxx, Uxx_plus_Vxx_inv);
    symmat_vec_mul(Uxx_plus_Vxx_inv, Ux_plus_Vx, Uxx_plus_Vxx_inv_Ux_plus_Vx, 1.0, 0.0);

    cost = U_plus_V - (1.0/2.0)*vec_dot(Ux_plus_Vx, Uxx_plus_Vxx_inv_Ux_plus_Vx);
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
    
    steps.resize(N+1);
    for(int k = 0; k <= N; k++){
        steps[k] = new DDPStep(this);
        steps[k]->k = k;
        steps[k]->Init();
    }
    for(int k = 0; k <= N; k++){
        steps[k]->prev = (k > 0 ? steps[k-1] : 0);
        steps[k]->next = (k < N ? steps[k+1] : 0);
    }

    fx      .resize(N);
	fu      .resize(N);
	fcor    .resize(N);
    fx_rev  .resize(N);
	fu_rev  .resize(N);
	fcor_rev.resize(N);
    for(int k = 0; k < N; k++){
        fx      [k].Allocate(nx, nx);
		fu      [k].Allocate(nx, nu);
		fcor    [k].Allocate(nx    );
        fx_rev  [k].Allocate(nx, nx);
        fu_rev  [k].Allocate(nx, nu);
        fcor_rev[k].Allocate(nx);    
    }
}

void DDPThread::Prepare(){
    int N  = solver->N;

    for(int k = 0; k < N; k++){
        mat_copy(solver->fx  [k], fx  [k]);
        mat_copy(solver->fu  [k], fu  [k]);
        vec_copy(solver->fcor[k], fcor[k]);

        mat_inv_gen(fx    [k], fx_rev[k]);
        mat_mat_mul(fx_rev[k], fu  [k], fu_rev  [k], -1.0, 0.0);
        mat_vec_mul(fx_rev[k], fcor[k], fcor_rev[k], -1.0, 0.0);
    }
}

void DDPThread::CalcValueForward(){
    int N = solver->N;

    for(int k = 0; k <= N; k++){
        steps[k]->CalcValueForward();
    }
}

void DDPThread::CalcValueBackward(){
    int N = solver->N;

    for(int k = N; k >= 0; k--){
        steps[k]->CalcValueBackward();
    }
}

void DDPThread::CalcStateForward(){
    int N = solver->N;

    for(int k = 0; k <= N; k++){
        steps[k]->CalcStateForward();
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

    Lx .Allocate(nx);
    Lu .Allocate(nu);
    Lxx.Allocate(nx, nx);
    Luu.Allocate(nu, nu);
    Lux.Allocate(nu, nx);
}

void DDPState::CalcCost(){

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

void CustomSolver::Shuffle(){
    vector< vector<DDPState*> > st_int;
    st_int.resize(N);
    vector< pair<int, DDPState*> >  idx;
    
    for(int k = 1; k <= N-1; k++){
        DDPState* stcur  = thread->steps[k  ]->state;
        DDPState* stprev = thread->steps[k-1]->state;
        DDPState* stnext = thread->steps[k+1]->state;
        st_int[k].resize(std::min(stprev->next.size(), stnext->prev.size()));
        vector<DDPState*>::iterator it = set_intersection(
            stprev->next.begin(), stprev->next.end(),
            stnext->prev.begin(), stnext->prev.end(),
            st_int[k].begin());
        st_int[k].resize(it - st_int[k].begin());
        st_int[k].erase(find(st_int[k].begin(), st_int[k].end(), stcur));

        for(DDPState* st : st_int[k])
            idx.push_back(make_pair(k, st));

    }

    //static std::default_random_engine urng(0);
    //shuffle(idx.begin(), idx.end(), urng);
    
    for(int i = 0; i < samples.size(); i++){
        int j = i%idx.size();
        samples[i]->k     = idx[j].first;
        samples[i]->state = idx[j].second;
        samples[i]->next  = thread->steps[samples[i]->k+1];
    }
}    

void CustomSolver::CompDP(){
    for(DDPState* st : stages[N]->states){
        st->Jopt    = (st->IsTerminal() ? st->L : 1000000000);
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

    DDPState* st = stages[0]->states[0];
    DSTR << "Jopt: " << st->Jopt << endl;
    while(st){
        thread->steps[st->stage->k]->state = st;
        st = st->nextOpt;
    }

}

void CustomSolver::CalcDirectionSearchDDP(){
    timer.CountUS();
    for(auto& con : cons_active)
		con->CalcCorrection();

    PrepareDDP ();

    int timePrepare1 = timer.CountUS();

    int nx = state[0]->dim;
    int nu = input[0]->dim;

    timer.CountUS();
    // calc cost
    for(int k = 0; k <= N; k++){
        for(DDPState* st : stages[k]->states){
            st->CalcCost();
        }
    }
    int timePrepare2 = timer.CountUS();

    const int numSample = 100;

    // for the first time, compute initial guess of mode sequence
    if(samples.empty()){
        thread = new DDPThread(this);
        thread->Init();

        samples.resize(numSample);
        for(int i = 0; i < numSample; i++){
            samples[i] = new DDPStep(thread);
            samples[i]->k = 0;
            samples[i]->Init();
        }

        CompDP();
        for(int k = 0; k <= N; k++)
            thread->steps[k]->state->Print();
        DSTR << endl;
    }
    
    timer.CountUS();
    real_t Jopt_prev = inf;
    while(true){
        // create shuffled sequences
        Shuffle();

        thread->Prepare();
        thread->CalcValueBackward();
        thread->CalcValueForward ();
        thread->CalcStateForward ();

        #pragma omp parallel for  num_threads(20)
        for(int i = 0; i < numSample; i++){
            // perform DDP with previous mode sequence
            samples[i]->CalcCost();
            DSTR << "sample " << i << ": " << samples[i]->cost << endl;
        }

        int    iopt = -1;
        real_t Jopt;
        for(int i = 0; i < numSample; i++){
            if(iopt == -1 || samples[i]->cost < Jopt){
                iopt = i;
                Jopt = samples[i]->cost;
            }
        }

        for(int k = 0; k <= N; k++)
            thread->steps[k]->state->Print();
        DSTR << Jopt << endl;
        
        if(Jopt_prev <= Jopt)
            break;

        samples[iopt]->Apply();
        Jopt_prev = Jopt;
    }
    int timeIter = timer.CountUS();

    DSTR << "prepare1: " << timePrepare1 << endl;
    DSTR << "prepare2: " << timePrepare2 << endl;
    DSTR << "iter    : " << timeIter     << endl;

    thread->CalcValueBackward();
    thread->CalcStateForward ();

    for(int k = 0; k <= N; k++)
        thread->steps[k]->state->Finish();

	for(int k = 0; k <= N; k++){
		for(SubState* subst : state[k]->substate){
			if(subst->var->locked)
				continue;

			int j0 = subst->index;
			for(int j = 0; j < subst->var->nelem; j++){
				subst->var->dx[j] = thread->steps[k]->dx(j0+j);
			}
		}
	}
	for(int k = 0; k < N; k++){
		for(SubInput* subin : input[k]->subinput){
			if(subin->var->locked)
				continue;

			int j0 = subin->index;
			for(int j = 0; j < subin->var->nelem; j++){
				subin->var->dx[j] = thread->steps[k]->du(j0+j);
			}
		}
		
	}
}

}
