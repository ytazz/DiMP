#pragma once

#include <DiMP/Types.h>

#include <set>

namespace DiMP{;

class CustomSolver : public Solver{
public:
    struct CustomMethod{
        enum{
            SearchDDP = Solver::Method::Major::Num,
        };
    };

    struct DDPNode : UTRefCount{
        DDPNode*       parent;
        CustomSolver*  solver;

        int     k;
        real_t  P;
	    vvec_t  Px;
	    vvec_t  Pu;
	    vmat_t  Pxx;
	    vmat_t  Puu;
	    vmat_t  Pux;
	    vmat_t  Puuinv;
	    vvec_t  Puuinv_Pu;
	    real_t  U;
	    vvec_t  Ux;
	    vmat_t  Uxx;
        real_t  cost;

        void Update();

        DDPNode();
    };

    class DDPCallback{
    public:
        virtual int       NumBranches(DDPNode* parent) = 0;
        virtual DDPNode*  CreateNode (DDPNode* parent, int idx) = 0;
    };

    struct CompByCost{
        bool operator()(const DDPNode* lhs, const DDPNode* rhs) const;
    };

    UTRef<DDPNode>    root;
    vector<DDPNode*>  nodes;
    set<DDPNode*, CompByCost>  queue;
    DDPCallback*      callback;

    vector<vmat_t>    fx_rev;
	vector<vmat_t>    fu_rev;
	vector<vvec_t>    f_cor_rev;

public:
    void CalcDirectionSearchDDP();

	virtual void    CalcDirection();
};

}
