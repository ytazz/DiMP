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
    Vector  Lx;
    Vector  Lu;
    Matrix  Lxx;
    Matrix  Luu;
    Matrix  Lux;
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
    
    int k;
    
             DDPStage(CustomSolver* _solver);
    virtual ~DDPStage();
};

class DDPThread;

class DDPStep : public UTRefCount{
public:
    DDPThread*  thread;

    int        k;
    DDPState*  state;
    real_t     cost;
    DDPStep*   next;
    DDPStep*   prev;
    
    Vector    dx;
    Vector    du;

    real_t    Q;
	Vector    Qx;
	Vector    Qu;
	Matrix    Qxx;
	Matrix    Quu;
	Matrix    Qux;
	Matrix    Quuinv;
	Vector    Quuinv_Qu;
    Matrix    Quuinv_Qux;
	real_t    V;
    Vector    Vx;
	Matrix    Vxx;
    Matrix    Vxxinv;
        
    real_t    P;
	Vector    Px;
	Vector    Pu;
	Matrix    Pxx;
	Matrix    Puu;
	Matrix    Pux;
	Matrix    Puuinv;
	Vector    Puuinv_Pu;
    Matrix    Puuinv_Pux;
	real_t    U;
	Vector    Ux;
	Matrix    Uxx;
    Matrix    Uxxinv;

    Vector    Vxx_fcor;
    Matrix    Vxx_fx;
    Matrix    Vxx_fu;
    Vector    Vx_plus_Vxx_fcor;
    Vector    Qu_plus_Qux_dx;

    Vector    Lx_plus_Ux;
    Matrix    Lxx_plus_Uxx;
    Vector    Lxx_plus_Uxx_fcor_rev;
    Vector    Lx_plus_Ux_plus_Lxx_plus_Uxx_fcor_rev;
    Matrix    Lxx_plus_Uxx_fx_rev;
    Matrix    Lxx_plus_Uxx_fu_rev;
    Matrix    Lux_fx_rev;

    real_t    U_plus_V;
    Vector    Ux_plus_Vx;
    Matrix    Uxx_plus_Vxx;
    Matrix    Uxx_plus_Vxx_inv;
    Vector    Uxx_plus_Vxx_inv_Ux_plus_Vx;
    
public:
    void  Init             ();
    void  Prepare          ();
    void  CalcValueForward ();
    void  CalcValueBackward();
    void  CalcStateForward ();
    void  CalcCost         ();
    void  Apply            ();

    DDPStep(DDPThread* _thread);
};

class DDPThread : public UTRefCount{
public:
    CustomSolver*  solver;
        
    vector<Matrix>    fx;
	vector<Matrix>    fu;
	vector<Vector>    fcor;
    vector<Matrix>    fx_rev;
	vector<Matrix>    fu_rev;
	vector<Vector>    fcor_rev;

    vector< UTRef<DDPStep> >  steps;

    void   Init             ();
    void   Prepare          ();
    void   CalcValueForward ();
    void   CalcValueBackward();
    void   CalcStateForward ();
        
             DDPThread(CustomSolver* _solver);
    virtual ~DDPThread();
};
    
class DDPCallback{
public:
    virtual DDPState* CreateInitialState() = 0;
	virtual void      CreateNextStates  (DDPState* _state, vector<DDPState*>& _next) = 0;
    virtual void      OnThreadUpdate    (DDPThread* _thread) = 0;
};

class CustomSolver : public Solver{
public:
    struct CustomMethod{
        enum{
            SearchDDP = Solver::Method::Major::Num,
        };
    };

    vector< UTRef<DDPStage > > stages;
    UTRef<DDPThread>           thread;
    vector< UTRef<DDPStep> >   samples;
    int                        numSample;

    DDPCallback*               callback;

public:
    void   CalcDirectionSearchDDP();
    void   Shuffle();
    void   CompDP ();
            
    virtual void    Init();
	virtual void    CalcDirection();

};

}
