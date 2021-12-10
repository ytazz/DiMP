#include <DiMP/Graph/Solver.h>

namespace DiMP{;

bool CustomSolver::CompByCost::operator()(const DDPNode* lhs, const DDPNode* rhs) const{
    return (lhs->cost <  rhs->cost) ||
           (lhs->cost == rhs->cost && lhs < rhs);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

CustomSolver::DDPNode::DDPNode(){

}

CustomSolver::DDPNode::~DDPNode(){

}

void CustomSolver::DDPNode::Resize(int nx, int nu){
    Lx              .resize(nx);
    Lu              .resize(nu);
    Lxx             .resize(nx, nx);
    Luu             .resize(nu, nu);
    Lux             .resize(nu, nx);
	Px              .resize(nx);
	Pu              .resize(nu);
	Pxx             .resize(nx, nx);
	Puu             .resize(nu, nu);
	Pux             .resize(nu, nx);
	Puuinv          .resize(nu, nu);
	Puuinv_Pu       .resize(nu);
	Ux              .resize(nx);
	Uxx             .resize(nx, nx);
    Ux_plus_Vx      .resize(nx);
    Uxx_plus_Vxx    .resize(nx, nx);
    Uxx_plus_Vxx_inv.resize(nx, nx);
}

void CustomSolver::DDPNode::Update(){
    P   =  parent->L  + parent->U
        + (parent->Lx + parent->Ux)*solver->f_cor_rev[k-1]
        + (1.0/2.0)*(solver->f_cor_rev[k-1]*((parent->Lxx + parent->Uxx)*solver->f_cor_rev[k-1]));
    Px  = solver->fx_rev[k-1].trans()*(parent->Lx  + parent->Ux + (parent->Lxx + parent->Uxx)*solver->f_cor_rev[k-1]);
    Pu  = solver->fu_rev[k-1].trans()*(parent->Lx  + parent->Ux + (parent->Lxx + parent->Uxx)*solver->f_cor_rev[k-1]) + parent->Lu;
    Pxx = solver->fx_rev[k-1].trans()*(parent->Lxx + parent->Uxx)*solver->fx_rev[k-1];
    Puu = solver->fu_rev[k-1].trans()*(parent->Lxx + parent->Uxx)*solver->fu_rev[k-1] + parent->Luu;
    Pux = solver->fu_rev[k-1].trans()*(parent->Lxx + parent->Uxx)*solver->fx_rev[k-1] + parent->Lux;

    int n = Puu.height();
	for(int i = 0; i < n; i++)
		Puu[i][i] += solver->param.regularization;
    	
	mat_inv_sym(Puu, Puuinv);

	Puuinv_Pu = Puuinv*Pu;

	U   = P   - (1.0/2.0)*(Pu*Puuinv_Pu);
	Ux  = Px  - Pux.trans()*Puuinv_Pu ;
	Uxx = Pxx - Pux.trans()*Puuinv*Pux;
		
    // enforce symmetry of Uxx
	int nx = Uxx.width();
	for(int i = 1; i < nx; i++) for(int j = 0; j < i; j++)
		Uxx[i][j] = Uxx[j][i];

    if(k < solver->N){
        // use relaxed cost-to-go as a lower bound
        const real_t alpha = 1.0;
        U_plus_V     = U   + alpha * solver->V  [k];
        Ux_plus_Vx   = Ux  + alpha * solver->Vx [k];
        Uxx_plus_Vxx = Uxx + alpha * solver->Vxx[k];
    }
    else{
        // add actual terminal cost
        U_plus_V     = U   + L  ;
        Ux_plus_Vx   = Ux  + Lx ;
        Uxx_plus_Vxx = Uxx + Lxx;
    }

    mat_inv_sym(Uxx_plus_Vxx, Uxx_plus_Vxx_inv);
    cost = U_plus_V - (1.0/2.0)*((Ux_plus_Vx)*(Uxx_plus_Vxx_inv*Ux_plus_Vx));

    //DSTR << "k " << k << "  V " << V[k] << "  Q " << Q[k] << endl;

}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CustomSolver::Init(){
    Solver::Init();

    if(param.methodMajor == CustomMethod::SearchDDP)
        InitDDP();
}

void CustomSolver::CalcDirection(){
    if(param.methodMajor == CustomMethod::SearchDDP){
		CalcDirectionSearchDDP();
    }
    else{
        Solver::CalcDirection();
    }
}

real_t CustomSolver::CalcObjective(){
    if(param.methodMajor == CustomMethod::SearchDDP){
        return CalcObjectiveSearchDDP();
    }
    else{
        return Solver::CalcObjective();
    }
}

void CustomSolver::CalcDirectionSearchDDP(){
    for(auto& con : cons_active)
		con->CalcCorrection();

    PrepareDDP ();
    BackwardDDP();

    fx_rev   .resize(N);
    fu_rev   .resize(N);
    f_cor_rev.resize(N);
    for(int k = 0; k < N; k++){
        mat_inv_gen(fx[k], fx_rev[k]);

        fu_rev   [k] = -fx_rev[k]*fu   [k];
        f_cor_rev[k] = -fx_rev[k]*f_cor[k];
    }

    nodes.clear();
    queue.clear();

    int nx = state[0]->dim;
    int nu = input[0]->dim;

    root    = callback->CreateNode(0, this, 0, nx, nu);
    root->k = 0;
 	root->U = 0.0;
	root->Ux .clear();
    root->Uxx.clear();
    root->cost = V[0];
    
    for(int i = 0; i < nx; i++)
	    root->Uxx[i][i] = 1000.0;

    nodes.push_back(root);
    queue.insert(root);

    nopt = 0;
    int neval = 0;

    while(!queue.empty()){
        DDPNode* n = *queue.begin();
        queue.erase(queue.begin());

        if(nopt && n->cost >= nopt->cost){
            //DSTR << "truncated" << endl;
            continue;
        }

        if(n->k == N){
            if(!nopt || n->cost < nopt->cost){
                nopt = n;
                //DSTR << "optimal node updated" << endl;
            }
            continue;
        }

        //DSTR << n->k << " " << n->cost << endl;

        // 
        int nbranch = callback->NumBranches(n);

        for(int i = 0; i < nbranch; i++){
            DDPNode* nchild = callback->CreateNode(n, this, i, nx, nu);
            nchild->Update();
            neval++;

            nodes.push_back(nchild);
            queue.insert(nchild);
        }
    }

    DSTR << "evaluated nodes: " << neval << endl;

    // calc dx and du backward
    dx[N] = -nopt->Uxx_plus_Vxx_inv*nopt->Ux_plus_Vx;
    //DSTR << N << " " << dx[N] << endl;

    DDPNode* n = nopt;
    while(n){
        callback->FinishNode(n);

        if(n->parent){
            du[n->k-1] = -n->Puuinv*(n->Pu + n->Pux*dx[n->k]);
            dx[n->k-1] = fx_rev[n->k-1]*dx[n->k] + fu_rev[n->k-1]*du[n->k-1] + f_cor_rev[n->k-1];

            //DSTR << n->k << " " << dx[n->k] << " " << du[n->k-1] << " " << dx[n->k-1] << endl;
        }

        n = n->parent;
    }

    // calc forward path to make sure dx[0] is zero
    //dx[0].clear();
    //for(int k = 0; k < N; k++){
    //    dx[k+1] = fx[k]*dx[k] + fu[k]*du[k] + f_cor[k];
    //}

	for(int k = 0; k <= N; k++){
		for(SubState* subst : state[k]->substate){
			if(subst->var->locked)
				continue;

			int j0 = subst->index;
			for(int j = 0; j < subst->var->nelem; j++){
				subst->var->dx[j] = dx[k][j0+j];
			}
		}
	}
	for(int k = 0; k < N; k++){
		for(SubInput* subin : input[k]->subinput){
			if(subin->var->locked)
				continue;

			int j0 = subin->index;
			for(int j = 0; j < subin->var->nelem; j++){
				subin->var->dx[j] = du[k][j0+j];
			}
		}
		
	}
}

real_t CustomSolver::CalcObjectiveSearchDDP(){
    for(auto& con : cons){
		if(!con->enabled)
			continue;
		
		con->CalcCoef ();
		con->CalcError();
	}

    real_t obj = 0.0;

    DDPNode* n = nopt;
    while(n){
        obj += callback->CalcNodeCost(n);
        n = n->parent;
    }

	return obj;

}

}
