#pragma once

/**
	Distributed Motion Solver Ver.2 (DiMP2)
 */

#include <DiMP2/Variable.h>
#include <DiMP2/Constraint.h>
#include <DiMP2/Link.h>

namespace DiMP2{;

/**
	最適化計算を行うクラス
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
			誤差eに関するzの最急降下方向へzを修正していく．
			y = c(z)の非線形の影響を受けるため修正率を小さく取れず，結果収束性が悪い．
		 */
		Steepest,

		/** 同時射影法
			- 各優先度レベルの拘束誤差を一定レートで減少させる変化量を求め，
			　これを上位優先度の拘束部分空間に射影する
		 */
		//Projection,
		
		/** 優先度つきPareto */
		Pareto,

		/** 動的過拘束検知.
			- 過拘束判定の閾値の設定が難しい
		 */
		//Dynamic,

		/** 拘束誤差の重み和を最小化 */
		Weighting,

		/** 高い優先度から段階的に最小化．LGPの定義に忠実 */
		Phased,

		numTypes
	};
};

/// iteration order modes
struct Iteration{
	enum{
		/// 拘束が作られた順
		Sequential,

		///	指定された起点から，拘束グラフに関して幅優先
		BreadthFirst,

		/// ランダム
		Random,
	};
};

/// logging modes
struct Logging{
	enum{
		/* Stepを一回呼ぶごとに一行増えるログ
			- 拘束種別誤差，優先度別誤差，所要反復回数
		 */
		MajorLoop,

		/* 一回のStep中のマイナループのログ
			- マイナループの一回の反復ごとに一行増える
			- 最大過拘束度，無効化された拘束リスト（;区切り）
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

	vector<real_t>		weights;		///< Weighting: 優先度別の重み
	vector<real_t>		minError;		///< Phased: 各優先度レベルで達成された最小誤差
	int					phase;			///< Phased: 現在最小化しているレベル

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
