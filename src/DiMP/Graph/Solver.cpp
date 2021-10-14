#include <DiMP/Graph/Solver.h>

namespace DiMP{;

bool CustomSolver::CompByCost::operator()(const DDPNode* lhs, const DDPNode* rhs) const{
    return lhs->cost < rhs->cost;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

CustomSolver::DDPNode::DDPNode(){

}

void CustomSolver::DDPNode::Update(){
    P   =  solver->L [k] + parent->U
        + (solver->Lx[k] + parent->Ux)*solver->f_cor_rev[k]
        + (1.0/2.0)*(solver->f_cor_rev[k]*((solver->Lxx[k] + parent->Uxx)*solver->f_cor_rev[k]));
    Px  = solver->fx_rev[k].trans()*(solver->Lx[k] + parent->Ux + (solver->Lxx[k] + parent->Uxx)*solver->f_cor_rev[k]);
    Pu  = solver->Lu [k]
        + solver->fu_rev[k].trans()*(solver->Lx[k] + parent->Ux + (solver->Lxx[k] + parent->Uxx)*solver->f_cor_rev[k]);
    Pxx = solver->fx_rev[k].trans()*(solver->Lxx[k] + parent->Uxx)*solver->fx_rev[k];
    Puu = solver->Luu[k]
        + solver->fu_rev[k].trans()*(solver->Lxx[k] + parent->Uxx)*solver->fu_rev[k];
    Pux = solver->Lux[k]
        + solver->fu_rev[k].trans()*(solver->Lxx[k] + parent->Uxx)*solver->fx_rev[k];

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

    //DSTR << "k " << k << "  V " << V[k] << "  Q " << Q[k] << endl;

}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CustomSolver::CalcDirection(){
    if(param.methodMajor == CustomMethod::SearchDDP){
        for(auto& con : cons_active)
			con->CalcCorrection();

		CalcDirectionSearchDDP();
    }
    else{
        Solver::CalcDirection();
    }

	// calc dx of dependent variables

}

void CustomSolver::CalcDirectionSearchDDP(){
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

    root      = new DDPNode();
    root->k   = 0;
 	root->U   = 0.0;
	root->Ux.clear();
    
    for(int i = 0; i < nx; i++)
	    root->Uxx[i][i] = 100.0;

    nodes.push_back(root);
    queue.insert(root);

    DDPNode* nopt = 0;

    while(!queue.empty()){
        DDPNode* n = *queue.begin();
        queue.erase(queue.begin());

        if(n->k == N){
            if(n->cost < nopt->cost){
                nopt = n;
            }
            return;
        }

        // 
        int nbranch = callback->NumBranches(n);

        for(int i = 0; i < nbranch; i++){
            DDPNode* nchild = callback->CreateNode(n, i);
            nchild->Update();

            nodes.push_back(nchild);
            queue.insert(nchild);
        }
    }

    // calc dx and du backward
    dx[N] = -nopt->Uxx.inv()*nopt->Ux;

    DDPNode* n = nopt->parent;
    while(n){
        du[n->k] = -n->Puuinv*(n->Pu + n->Pux*dx[n->k+1]);
        dx[n->k] = fx_rev[n->k]*dx[n->k+1] + fu_rev[n->k]*du[n->k] + f_cor_rev[n->k];

        n = n->parent;
    }

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

}
