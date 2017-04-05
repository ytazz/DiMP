#pragma once

/**
	Distributed Motion Solver Ver.2 (DiMP2)
 */

#include <DiMP2/Variable.h>
#include <DiMP2/Constraint.h>
#include <DiMP2/Link.h>

namespace DiMP2{;

/**
	�œK���v�Z���s���N���X
 */

typedef vector< Variable* >				Variables;
typedef vector< UTRef<Variable> >		VariableRefs;
typedef vector< Constraint* >			Constraints;
typedef vector< UTRef<Constraint> >		ConstraintRefs;
typedef vector< UTRef<Link> >			LinkRefs;

class Solver;

class Subgraph{
public:
	Variables	vars;
	Constraints	cons;
	Constraint*	root;
};

/// algorithm types
struct Algorithm{
	enum{
		/**
			�덷e�Ɋւ���z�̍ŋ}�~��������z���C�����Ă����D
			y = c(z)�̔���`�̉e�����󂯂邽�ߏC��������������ꂸ�C���ʎ������������D
		 */
		Steepest,

		/** �����ˉe�@
			- �e�D��x���x���̍S���덷����背�[�g�Ō���������ω��ʂ����߁C
			�@�������ʗD��x�̍S��������ԂɎˉe����
		 */
		//Projection,
		
		/** �D��x��Pareto */
		Pareto,

		/** ���I�ߍS�����m.
			- �ߍS�������臒l�̐ݒ肪���
		 */
		//Dynamic,

		/** �S���덷�̏d�ݘa���ŏ��� */
		Weighting,

		/** �����D��x����i�K�I�ɍŏ����DLGP�̒�`�ɒ��� */
		Phased,

		numTypes
	};
};

/// iteration order modes
struct Iteration{
	enum{
		/// �S�������ꂽ��
		Sequential,

		///	�w�肳�ꂽ�N�_����C�S���O���t�Ɋւ��ĕ��D��
		BreadthFirst,

		/// �����_��
		Random,
	};
};

/// logging modes
struct Logging{
	enum{
		/* Step�����ĂԂ��ƂɈ�s�����郍�O
			- �S����ʌ덷�C�D��x�ʌ덷�C���v������
		 */
		MajorLoop,

		/* ����Step���̃}�C�i���[�v�̃��O
			- �}�C�i���[�v�̈��̔������ƂɈ�s������
			- �ő�ߍS���x�C���������ꂽ�S�����X�g�i;��؂�j
		 */
		MinorLoop,
		numModes
	};
};

class Graph;

/**
	Constraint Solver
 */
class Solver : public UTRefCount{
public:
	Graph* graph;

	/** number of iterations of Gauss-Seidel method.
		Generally, residual constraint error decreases exponentially w.r.t. number of iterations,
		while the computation time grows linearly.
	 */
	
	/**	number of G-S iterations for error-correction of this level
		*/
	vector<uint>	maxIter;
	uint			maxIterDefault;
	
	/** algorithm type
	 */
	int		algo;

	/** iteration order
	 */
	int		order;

	/// error threshould of minor loop
	real_t	epsAbs, epsAbs2;
	real_t	epsRel, epsRel2;

	/// step size
	real_t  stepSizeMin;
	real_t  stepSizeMax;

	bool	ready;

public:
	uint	count;						///< counter that increments at every call of Step
	
	VariableRefs		vars;			///< array of all variables
	
	ConstraintRefs		cons;			///< array of all constraints
	//vector<Constraints>	cons_arranged;
	vector<Constraints>	cons_level;

	uint				maxLevel;		///< maximum priority level
	
	LinkRefs			links;			///< array of links
	LinkRefs			links_proxy;

	real_t	e;							///< sum of all constraint errors
	vector<real_t>		e_type;			///< sum for each constraint category
	vector<real_t>		e_level;		///< sum for each priority level

	vector<real_t>		weights;		///< Weighting: �D��x�ʂ̏d��
	vector<real_t>		minError;		///< Phased: �e�D��x���x���ŒB�����ꂽ�ŏ��덷
	int					phase;			///< Phased: ���ݍŏ������Ă��郌�x��

	/// logging
	bool		doLog[Logging::numModes];
	ofstream	file [Logging::numModes];
	
public: /// solver-specific variables and functions
	/// steepest descent
	real_t CalcStepSize(int level);
	real_t CalcCost    (real_t alpha, int level);
	
	//void   StepSteepest ();
	void   StepPareto   ();
	//void   StepWeighting();
	//void   StepPhased   ();
	
	/// projection
	//void IterateProjection(uint L, bool corr_or_proj);
	//void StepProjection();
	//void StepDynamic();

public: /// internal functions

	void ComposeLink(Link* l0, Link* l1);

	/// add variable
	void AddVar(Variable* var);

	/// add constraint
	void AddCon(Constraint* con);

	/// add link
	void AddLink     (Link* link);
	//void AddLinkProxy(Link* link);

	/// logging
	void LogLabel(int mode);
	void LogValue(int mode);

public: /// public functions called by Graph or by user

	/** set number of iterations
		@param n		# of iterations
		@param l		priority level
		@param initial	true: error-correction, false: projection
	 */
	void SetNumIteration(uint n, uint l = 0);

	/// do initialization
	void Init();

	/// set solver type
	void SetAlgorithm(int t);

	/// set iteration order
	void SetIterationOrder(int o);

	/// set tolerance of minor loop
	void SetTolerance(real_t eps_abs, real_t eps_rel);

	/// enable/disable logging
	void EnableLogging(int mode, bool on = true);

	/// one step update
	void Step();

	/// deletes all variables and constraints
	void Clear();

	/// resets all variables
	void Reset();

	Solver();

};

}
