#pragma once

#include <DiMP/Types.h>

#include <sbblas.h>

#include <set>
#include <deque>

namespace DiMP{;

class CustomSolver;
class DDPStage;

class DDPState : public UTRefCount{
public:
    CustomSolver*      solver;
    DDPStage*          stage;
    set<DDPState*>     next;
    set<DDPState*>     prev;
    DDPState*          nextOpt;

    real_t  L;
        
    vvec_t  Lx;
    vvec_t  Lu;
    vmat_t  Lxx;
    vmat_t  Luu;
    vmat_t  Lux;
        
    Vector  _Lx;
    Vector  _Lu;
    Matrix  _Lxx;
    Matrix  _Luu;
    Matrix  _Lux;
    //Vector  _Lxx_dx;
    //Vector  _Luu_du;
    //Vector  _Lux_dx;

    //real_t  J;
    real_t  Jopt;

    virtual bool IsIdentical(const DDPState* st) = 0;
    virtual bool IsTerminal () = 0;
    virtual void Init       ();
    virtual void CalcCost   ();
    virtual void Finish     ();
    virtual void Print      ();

                DDPState(CustomSolver* _solver);
    virtual ~DDPState();
};

class DDPStage : public UTRefCount{
public:
    CustomSolver*  solver;
    DDPStage*      next;
    vector< UTRef<DDPState> >  states;
    
    int     k;
    Vector  fcor;
    Matrix  fx;
    Matrix  fu;

    void Init    ();
    void Prepare ();

                DDPStage(CustomSolver* _solver);
    virtual ~DDPStage();
};

class DDPThread : public UTRefCount{
public:
    CustomSolver*  solver;
        
    vector<DDPState*>  path;

    vector<Vector>  dx;
    vector<Vector>  du;

    vector<real_t>  Q;
	vector<Vector>  Qx;
	vector<Vector>  Qu;
	vector<Matrix>  Qxx;
	vector<Matrix>  Quu;
	vector<Matrix>  Qux;
	vector<Matrix>  Quuinv;
	vector<Vector>  Quuinv_Qu;
    vector<Matrix>  Quuinv_Qux;
	vector<real_t>  V;
	vector<Vector>  Vx;
	vector<Matrix>  Vxx;
    vector<Matrix>  Vxxinv;
        
    vector<Vector>  Vxx_fcor;
    vector<Matrix>  Vxx_fx;
    vector<Matrix>  Vxx_fu;
    vector<Vector>  Vx_plus_Vxx_fcor;
    vector<Vector>  Qu_plus_Qux_dx;

    void   Init    ();
    void   Backward();
    void   Forward ();
        
                DDPThread(CustomSolver* _solver);
    virtual ~DDPThread();
};
    
class DDPCallback{
public:
    virtual DDPState* CreateInitialState() = 0;
	virtual void      CreateNextStates  (DDPState* _state, vector<DDPState*>& _next) = 0;
};

class CustomSolver : public Solver{
public:
    struct CustomMethod{
        enum{
            SearchDDP = Solver::Method::Major::Num,
        };
    };

    vector< UTRef<DDPStage > > stages;
    vector< UTRef<DDPThread> > threads;

    DDPCallback*               callback;

public:
    void   CalcDirectionSearchDDP();
    //void   Shuffle(const vector<DDPState*>& path0, vector<DDPState*>& path1);
    void   Shuffle();
    void   CompDP (vector<DDPState*>& path);
            
    virtual void    Init();
	virtual void    CalcDirection();

};

}
