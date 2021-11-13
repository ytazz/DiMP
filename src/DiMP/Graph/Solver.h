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

    class DDPNode : public UTRefCount{
    public:
        DDPNode*       parent;
        CustomSolver*  solver;

        int     k;
        real_t  L;
        vvec_t  Lx;
        vvec_t  Lu;
        vmat_t  Lxx;
        vmat_t  Luu;
        vmat_t  Lux;
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
        real_t  U_plus_V;
        vvec_t  Ux_plus_Vx;
        vmat_t  Uxx_plus_Vxx;
        vmat_t  Uxx_plus_Vxx_inv;
        real_t  cost;

    public:
        void Resize(int nx, int nu);
        void Update();

                 DDPNode();
        virtual ~DDPNode();
    };

    class DDPCallback{
    public:
        virtual int       NumBranches (DDPNode* _parent) = 0;
        virtual DDPNode*  CreateNode  (DDPNode* _parent, CustomSolver* _solver, int idx, int nx, int nu) = 0;
        virtual void      FinishNode  (DDPNode* _node) = 0;
        virtual real_t    CalcNodeCost(DDPNode* _node) = 0;
    };

    struct CompByCost{
        bool operator()(const DDPNode* lhs, const DDPNode* rhs) const;
    };

    DDPNode*                   root;
    vector< UTRef<DDPNode> >   nodes;
    set<DDPNode*, CompByCost>  queue;
    DDPNode*          nopt;
    DDPCallback*      callback;

    vector<vmat_t>    fx_rev;
	vector<vmat_t>    fu_rev;
	vector<vvec_t>    f_cor_rev;

public:
    void   CalcDirectionSearchDDP();
    real_t CalcObjectiveSearchDDP();

    virtual void    Init();
	virtual void    CalcDirection();
    virtual real_t  CalcObjective();

};

}
