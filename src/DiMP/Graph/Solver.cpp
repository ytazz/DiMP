#include <DiMP/Graph/Solver.h>
#include <DiMP/Graph/Graph.h>

#include <sbtimer.h>
#include <sbrandom.h>

#include <algorithm>
#include <random>

namespace DiMP{;

static const real_t inf = numeric_limits<real_t>::max();
static Timer timer;
static Timer timer2;

const real_t valueEps = 1.0e10;

///////////////////////////////////////////////////////////////////////////////////////////////////

DDPAutomaton::DDPAutomaton(CustomSolver* _solver){
    solver = _solver;
}

DDPAutomaton::~DDPAutomaton(){

}

///////////////////////////////////////////////////////////////////////////////////////////////////

DDPStep::DDPStep(DDPThread* _thread){
    thread = _thread;
}

void DDPStep::Init(){
    int N  = thread->solver->N;
    int nx  = thread->solver->state[k]->dim;
    int nu  = thread->solver->input[k]->dim;
    
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
        Lux_fu_rev                           .Allocate(nu, nu);
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
    state->CalcCost(k);

    if(!prev){
        U = 0.0;
        vec_clear(Ux );
        mat_clear(Uxx);

        Ud = 0.0;
    }
    if(next){
        int nx = thread->solver->fx[k].n;

        vec_copy   (state->Lx[k] , Lx_plus_Ux  );
        vec_add    (Ux           , Lx_plus_Ux  );
        mat_copy   (state->Lxx[k], Lxx_plus_Uxx);
        mat_add    (Uxx          , Lxx_plus_Uxx);
        mat_vec_mul(Lxx_plus_Uxx, thread->solver->fcor_rev[k], Lxx_plus_Uxx_fcor_rev, 1.0, 0.0);
        vec_copy   (Lx_plus_Ux           , Lx_plus_Ux_plus_Lxx_plus_Uxx_fcor_rev);
        vec_add    (Lxx_plus_Uxx_fcor_rev, Lx_plus_Ux_plus_Lxx_plus_Uxx_fcor_rev);
        mat_mat_mul(Lxx_plus_Uxx, thread->solver->fx_rev[k], Lxx_plus_Uxx_fx_rev, 1.0, 0.0);
        mat_mat_mul(Lxx_plus_Uxx, thread->solver->fu_rev[k], Lxx_plus_Uxx_fu_rev, 1.0, 0.0);
        mat_mat_mul(state->Lux[k], thread->solver->fx_rev[k], Lux_fx_rev, 1.0, 0.0);
        mat_mat_mul(state->Lux[k], thread->solver->fu_rev[k], Lux_fu_rev, 1.0, 0.0);

        P = state->L[k] + U + vec_dot(Lx_plus_Ux, thread->solver->fcor_rev[k]) + (1.0/2.0)*vec_dot(thread->solver->fcor_rev[k], Lxx_plus_Uxx_fcor_rev);
        
        mattr_vec_mul(thread->solver->fx_rev[k], Lx_plus_Ux_plus_Lxx_plus_Uxx_fcor_rev, Px, 1.0, 0.0);
        
        vec_copy(state->Lu[k], Pu);
        mattr_vec_mul(thread->solver->fu_rev[k], Lx_plus_Ux_plus_Lxx_plus_Uxx_fcor_rev, Pu, 1.0, 1.0);
        
        mattr_mat_mul(thread->solver->fx_rev[k], Lxx_plus_Uxx_fx_rev, Pxx, 1.0, 0.0);
        if(thread->solver->param.useHessian){
            for(int i = 0; i < nx; i++)
                mat_add(thread->solver->fxx_rev[k][i], Pxx, Lx_plus_Ux_plus_Lxx_plus_Uxx_fcor_rev(i));
        }

        mat_copy (state->Luu[k], Puu);
        mat_add  (Lux_fu_rev, Puu);
        mattr_add(Lux_fu_rev, Puu);
        mattr_mat_mul(thread->solver->fu_rev[k], Lxx_plus_Uxx_fu_rev, Puu, 1.0, 1.0);
        if(thread->solver->param.useHessian){
            for(int i = 0; i < nx; i++)
                mat_add(thread->solver->fuu_rev[k][i], Puu, Lx_plus_Ux_plus_Lxx_plus_Uxx_fcor_rev(i));
        }

        mat_copy(Lux_fx_rev, Pux);
        mattr_mat_mul(thread->solver->fu_rev[k], Lxx_plus_Uxx_fx_rev, Pux, 1.0, 1.0);
        if(thread->solver->param.useHessian){
            for(int i = 0; i < nx; i++)
                mat_add(thread->solver->fux_rev[k][i], Pux, Lx_plus_Ux_plus_Lxx_plus_Uxx_fcor_rev(i));
        }

        for(int i = 0; i < Puu.m; i++)
            Puu(i,i) += thread->solver->reg_u;
    	
	    mat_inv_pd(Puu, Puuinv);

        Matrix test;
        test.Allocate(Puu.m, Puu.n);
        symmat_mat_mul(Puuinv, Puu, test, 1.0, 0.0);

        symmat_vec_mul(Puuinv, Pu , Puuinv_Pu , 1.0, 0.0);
	    symmat_mat_mul(Puuinv, Pux, Puuinv_Pux, 1.0, 0.0);
        
	    next->U = P - (1.0/2.0)*vec_dot(Pu, Puuinv_Pu);
	    
        vec_copy(Px, next->Ux);
        mattr_vec_mul(Pux, Puuinv_Pu, next->Ux, -1.0, 1.0);
	    
        mat_copy(Pxx, next->Uxx);
        mattr_mat_mul(Pux, Puuinv_Pux, next->Uxx, -1.0, 1.0);
		
        // enforce symmetry of Uxx
	    for(int i = 1; i < next->Uxx.m; i++) for(int j = 0; j < i; j++)
		    next->Uxx(i,j) = next->Uxx(j,i);

        //
        next->Ud = Ud + thread->solver->callback->CalcTransitionCost(state, next->state, k);
    }

    // add terminal cost
    //U  [N] += L  [N];
    //Ux [N] += Lx [N];
    //Uxx[N] += Lxx[N];
    //mat_inv_sym(Uxx[N], Uxxinv);
}

void DDPStep::CalcValueBackward(){
    state->CalcCost(k);

    if(!next){
        V = state->L[k];
        vec_copy(state->Lx [k], Vx );
        mat_copy(state->Lxx[k], Vxx);
        
        for(int i = 0; i < Vxx.m; i++)
            Vxx(i,i) += thread->solver->reg_x;
    	
        Vd = 0.0;
    }
    else{
        int nx1 = thread->solver->fx[k].m;
        
        mat_vec_mul(next->Vxx, thread->solver->fcor[k], Vxx_fcor, 1.0, 0.0);
        mat_mat_mul(next->Vxx, thread->solver->fx  [k], Vxx_fx  , 1.0, 0.0);
        mat_mat_mul(next->Vxx, thread->solver->fu  [k], Vxx_fu  , 1.0, 0.0);
        vec_copy(next->Vx, Vx_plus_Vxx_fcor);
        vec_add (Vxx_fcor, Vx_plus_Vxx_fcor);

        Q = state->L[k] + next->V + vec_dot(next->Vx, thread->solver->fcor[k]) + (1.0/2.0)*vec_dot(thread->solver->fcor[k], Vxx_fcor);
        
        vec_copy(state->Lx[k], Qx);
        mattr_vec_mul(thread->solver->fx[k], Vx_plus_Vxx_fcor, Qx, 1.0, 1.0);

        vec_copy(state->Lu[k], Qu);
        mattr_vec_mul(thread->solver->fu[k], Vx_plus_Vxx_fcor, Qu, 1.0, 1.0);

        mat_copy(state->Lxx[k], Qxx);
        for(int i = 0; i < Qxx.m; i++)
            Qxx(i,i) += thread->solver->reg_x;

        mattr_mat_mul(thread->solver->fx[k], Vxx_fx, Qxx, 1.0, 1.0);
        if(thread->solver->param.useHessian){
            for(int i = 0; i < nx1; i++)
                mat_add(thread->solver->fxx[k][i], Qxx, Vx_plus_Vxx_fcor(i));
        }

        mat_copy(state->Luu[k], Quu);
        mattr_mat_mul(thread->solver->fu[k], Vxx_fu, Quu, 1.0, 1.0);
        if(thread->solver->param.useHessian){
            for(int i = 0; i < nx1; i++)
                mat_add(thread->solver->fuu[k][i], Quu, Vx_plus_Vxx_fcor(i));
        }

        mat_copy(state->Lux[k], Qux);
        mattr_mat_mul(thread->solver->fu[k], Vxx_fx, Qux, 1.0, 1.0);
        if(thread->solver->param.useHessian){
            for(int i = 0; i < nx1; i++)
                mat_add(thread->solver->fux[k][i], Qux, Vx_plus_Vxx_fcor(i));
        }

        for(int i = 0; i < Quu.m; i++)
            Quu(i,i) += thread->solver->reg_u;
    	
        mat_inv_pd(Quu, Quuinv);

        Matrix test;
        test.Allocate(Quu.m, Quu.n);
        symmat_mat_mul(Quuinv, Quu, test, 1.0, 0.0);
        
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

        Vd = next->Vd + thread->solver->callback->CalcTransitionCost(state, next->state, k);
    }
}

void DDPStep::CalcStateForward(real_t alpha){
    if(!prev){
        mat_inv_pd(Vxx, Vxxinv);
        symmat_vec_mul(Vxxinv, Vx, dx, -alpha, 0.0);
        //vec_clear(dx);
    }

    if(next){
        vec_copy(Qu, Qu_plus_Qux_dx);
        for(int i = 0; i < Qu_plus_Qux_dx.n; i++)
			Qu_plus_Qux_dx(i) *= alpha;

        mat_vec_mul(Qux, dx, Qu_plus_Qux_dx, 1.0, 1.0);
        symmat_vec_mul(Quuinv, Qu_plus_Qux_dx, du, -1.0, 0.0);

        vec_copy   (thread->solver->fcor[k], next->dx);
        mat_vec_mul(thread->solver->fx  [k], dx, next->dx, 1.0, 1.0);
        mat_vec_mul(thread->solver->fu  [k], du, next->dx, 1.0, 1.0);
        if(thread->solver->param.useHessian){
            int nx1 = next->dx.n;
            for(int i = 0; i < nx1; i++){
                next->dx(i) += (1.0/2.0)*quadform(thread->solver->fxx[k][i], dx, dx);
                next->dx(i) += (1.0/2.0)*quadform(thread->solver->fuu[k][i], du, du);
                next->dx(i) +=           quadform(thread->solver->fux[k][i], du, dx);
            }
        }

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

    cost += (thread->steps[k]->Ud + Vd);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

DDPThread::DDPThread(CustomSolver* _solver){
    solver = _solver;
}

DDPThread::~DDPThread(){

}

void DDPThread::Init(){
    int N  = solver->N;
    
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
}

bool DDPThread::CalcValueForward(){
    int N = solver->N;

    for(int k = 0; k <= N; k++){
        steps[k]->CalcValueForward();
        //DSTR << "U : " << k << " " << steps[k]->U << endl;

        if(steps[k]->U < -valueEps)
            return false;
    }

    return true;
}

bool DDPThread::CalcValueBackward(){
    int N = solver->N;

    for(int k = N; k >= 0; k--){
        steps[k]->CalcValueBackward();
        DSTR << "V : " << k << " " << steps[k]->V << endl;

        if(steps[k]->V < -valueEps)
            return false;
    }

    return true;
}

void DDPThread::CalcStateForward(real_t alpha){
    int N = solver->N;

    for(int k = 0; k <= N; k++){
        steps[k]->CalcStateForward(alpha);
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

    nextOpt.resize(solver->N+1);
    Jopt   .resize(solver->N+1);

    L  .resize(solver->N+1);
    Lx .resize(solver->N+1);
    Lu .resize(solver->N+1);
    Lxx.resize(solver->N+1);
    Luu.resize(solver->N+1);
    Lux.resize(solver->N+1);

    for(int k = 0; k <= solver->N; k++){
        Lx [k].Allocate(nx);
        Lu [k].Allocate(nu);
        Lxx[k].Allocate(nx, nx);
        Luu[k].Allocate(nu, nu);
        Lux[k].Allocate(nu, nx);
    }
}

void DDPState::Print(){

}

void DDPState::CalcCost(int k){

}

void DDPState::Finish(int k){

}
	
///////////////////////////////////////////////////////////////////////////////////////////////////

void CustomSolver::Init(){
    Solver::Init();

    if(param.methodMajor == CustomMethod::SearchDDP){
        InitDDP();

        // create automaton
        automaton = new DDPAutomaton(this);

        stIni = callback->CreateInitialState();
        stIni->Init();
        automaton->states.push_back(stIni);

        deque<DDPState*> queue;
        queue.push_back(stIni);

        vector< UTRef<DDPState> > _next;
        DDPState* st0;
        DDPState* st1;
            
        while(!queue.empty()){
            st0 = queue.front();
            queue.pop_front();

            _next.clear();
            callback->CreateNextStates(st0, _next);
                
            for(DDPState* _st1 : _next){
                st1 = 0;
                for(DDPState* __st1 : automaton->states){
                    if(__st1->IsIdentical(_st1)){
                        st1 = __st1;
                        break;
                    }
                }
                if(!st1){
                    automaton->states.push_back(_st1);
                    queue.push_back(_st1);
                    st1 = _st1;
                    st1->Init();
                }

                st0->next.push_back(st1);
                st1->prev.push_back(st0);
            }
        }

        // assign unique id
        int id = 0;
        for(DDPState* st : automaton->states)
            st->id = id++;

        // sort next and prev by id
        auto cmp = [](DDPState* lhs, DDPState* rhs){ return lhs->id < rhs->id; };
        for(DDPState* st : automaton->states){
            sort(st->next.begin(), st->next.end(), cmp);
            sort(st->prev.begin(), st->prev.end(), cmp);
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

real_t CustomSolver::CalcObjective(){
    graph->Prepare();

    return Solver::CalcObjective();
}

void CustomSolver::ModifyVariables(real_t alpha){
    if(param.methodMajor == CustomMethod::SearchDDP){
        thread->CalcStateForward(1.0);

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

        for(auto& var : vars_unlocked)
		    var->Modify(alpha);
    }
    else{
        Solver::ModifyVariables(alpha);
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
            st_int[k].begin(),
            [](DDPState* lhs, DDPState*rhs){ return lhs->id < rhs->id; }
        );
        st_int[k].resize(it - st_int[k].begin());
        st_int[k].erase(find(st_int[k].begin(), st_int[k].end(), stcur));

        for(DDPState* st : st_int[k])
            idx.push_back(make_pair(k, st));

    }

    numSample = std::min(idx.size(), samples.size());
    DSTR << "num sample: " << idx.size() << " " << numSample << endl;
    
    for(int i = 0; i < numSample; i++){
        samples[i]->k     = idx[i].first;
        samples[i]->state = idx[i].second;
        samples[i]->next  = thread->steps[samples[i]->k+1];
    }
}    

void CustomSolver::CompDP(){
    for(DDPState* st : automaton->states){
        st->CalcCost(N);
        st->Jopt[N]    = (st->IsTerminal() ? st->L[N] : 1000000000);
        st->nextOpt[N] = 0;
    }

    for(int k = N-1; k >= 0; k--){
        for(DDPState* st0 : automaton->states){
            st0->nextOpt[k] = 0;
            st0->Jopt[k] = inf;

            st0->CalcCost(k);
            
            for(DDPState* st1 : st0->next){
                real_t J = callback->CalcTransitionCost(st0, st1, k) + st1->Jopt[k+1];

                // compare by id if cost is exactly the same
                if(!st0->nextOpt[k] || J < st0->Jopt[k] || (J == st0->Jopt[k] && st1->id < st0->nextOpt[k]->id)){
                    st0->nextOpt[k] = st1;
                    st0->Jopt[k] = J;
                }
            }
        }
    }

    //DDPState* st = stages[0]->states[0];
    DDPState* st = stIni;
    //DSTR << "Jopt: " << st->Jopt << endl;
    for(int k = 0; k <= N; k++){
        thread->steps[k]->state = st;
        st = st->nextOpt[k];
    }

}

void CustomSolver::CalcDirectionSearchDDP(){
    timer.CountUS();
    for(auto& con : cons_active)
		con->CalcCorrection();

    CalcTransitionDDP();
    CalcReverseTransitionDDP();
    CalcCostDDP();
    CalcCostGradientDDP();

    int timePrepare1 = timer.CountUS();

    const int numSampleMax = 1000;

    // for the first time, compute initial guess of mode sequence
    if(samples.empty()){
        thread = new DDPThread(this);
        thread->Init();

        samples.resize(numSampleMax);
        for(int i = 0; i < numSampleMax; i++){
            samples[i] = new DDPStep(thread);
            samples[i]->k = 0;
            samples[i]->Init();
        }

        CompDP();
        //for(int k = 0; k <= N; k++)
        //    thread->steps[k]->state->Print();
        //DSTR << endl;
    }
    
    real_t Jopt_prev = inf;
    real_t Jopt;
    int subiterCount = 0;

    int timeIter[4] = {0,0,0,0};
    int retryCount = 0;
    while(true){
        // create shuffled sequences
        timer.CountUS();
        Shuffle();
        timeIter[0] += timer.CountUS();

        timer.CountUS();
        reg_u = param.regularization;
        while(true){
            bool ret[2];
            #pragma omp parallel sections if(param.parallelize)
            {
                #pragma omp section
                ret[0] = thread->CalcValueBackward();
                #pragma omp section
                ret[1] = thread->CalcValueForward ();
            }

            if(ret[0] && ret[1])
                break;

            reg_u *= 10.0;
            retryCount++;
        }
        timeIter[1] += timer.CountUS();

        timer.CountUS();
        #pragma omp parallel for if(param.parallelize)
        for(int i = 0; i < numSample; i++){
            samples[i]->CalcCost();
            /*
            DSTR << "sample " << i << ": "
                 << " k: "     << samples[i]->k
                 << " state: ";
                 samples[i]->state->Print();
            DSTR << " J: "     << samples[i]->cost
                 << endl;
            */
        }
        timeIter[2] += timer.CountUS();

        int    iopt = -1;
        for(int i = 0; i < numSample; i++){
            if( iopt == -1 || 
                 samples[i]->cost <  Jopt || 
                (samples[i]->cost == Jopt && samples[i]->k <  samples[iopt]->k) ||
                (samples[i]->cost == Jopt && samples[i]->k == samples[iopt]->k && samples[i]->state->id < samples[iopt]->state->id)
            ){
                iopt = i;
                Jopt = samples[i]->cost;
            }
        }

        //for(int k = 0; k <= N; k++)
        //    thread->steps[k]->state->Print();
        //DSTR << Jopt << endl;

        callback->OnThreadUpdate(thread);
        
        const real_t Jeps = 0.0;
        if(Jopt_prev <= Jopt + Jeps)
            break;

        samples[iopt]->Apply();
        Jopt_prev = Jopt;

        //break;
    }
    
    //for(int k = 0; k <= N; k++){
    //    DSTR << k << ": ";
    //    thread->steps[k]->state->Print();
    //}
    //DSTR << Jopt << endl;

    timer.CountUS();
    thread->CalcValueBackward();
    //thread->CalcStateForward ();

    for(int k = 0; k <= N; k++)
        thread->steps[k]->state->Finish(k);

    timeIter[3] += timer.CountUS();

    DSTR << "prepare1: " << timePrepare1 << endl;
    DSTR << "iter0   : " << timeIter[0]  << endl;
    DSTR << "iter1   : " << timeIter[1]  << endl;
    DSTR << "iter2   : " << timeIter[2]  << endl;
    DSTR << "iter3   : " << timeIter[3]  << endl;
    DSTR << "retry   : " << retryCount   << endl;

}

}
