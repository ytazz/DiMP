class  RevRangeConS;
class  MidRangeConS;
class  RevMoveConS;

	RevRangeConS*	con_proh_cop;		///< (add) step prohibited area 
	MidRangeConS*	con_proh_mid;		///< (add)
	RevMoveConS*	con_move_cop;		///< (add) moving constraint for cop

		real_t control_period ;			///<実際の制御周期
		real_t calc_period ;			///<計算上の周期

		real_t start_move_time;			///< 最初に初期位置まで足先を持ってくる時間
		real_t foot_place;				///< 重心からの初期足先位置（足の広げ）
		vec2_t rfoot_place;				///< 重心からの初期右足先位置
		vec2_t lfoot_place;				///< 重心からの初期左足先位置
		
		vec2_t proh_pos;				///< 接地不可中心
		real_t proh_r;					///< 接地不可領域半径


		vec2_t initial_velocity;		///< 重心初速度

		real_t max_swing_height;			///< 遊脚高さの最大値

		bool out_csv;					///<モーションデータを出力するか
		bool use_beginning;				///<初期状態を与えるか
		bool use_terminal_pos ;			///<終端条件を与えるか
		bool use_terminal_vel ;			///<終端速度を与えるか
		bool out_enuvo;					///<enuvoの共有メモリに出力するか
		bool online;					///<オンライン制御するか
		bool use_initial_trajectory;	///<初期軌道を与えるか

		std::map<int, int>	phase;		///< walking phase at each step

		//モーションデータ作成時に使用
		real_t alpha;					/// 足質量×alpha＝胴体質量 
		real_t beta;					/// 胴体中心からの足の重心距離×beta ＝ 実際の足の長さ
		real_t real_height;				/// 腰の高さ

		Param();
	};
	/// 経由点
	struct Waypoint{
		int     k;
		vec2_t  pos;
		vec2_t  vel;
	};
	/// 軌道
	struct TrajPoint{
		float  t;
		Vec3f  pcom;
		Vec3f  ptorso;
		Vec3f  pcop;
		Vec3f  pswing;
	};

	//(add) prohibited area
	struct ProhArea{
		vec2_t proh_cop;
		real_t r;
	};

	//(add) prohibited moving area
	struct MoveArea{
		vec2_t proh_move;
		vec2_t vel_move;
		real_t r;
	};


	//(add) prohibited area
	void AddProhArea	(vec2_t proh_cop, real_t proh_r);
	vector<ProhArea>	prohareas;

	//(add) prohibited moving area
	void AddMoveArea	(vec2_t proh_move, real_t proh_r,vec2_t vel_move);
	vector<MoveArea>	moveareas;

	
	bool move_trajectory;
	float tm;	// 描写時間
	clock_t start,test;

	Vec3f pos_right, pos_left;


// test add 11/19 (将来的にはBipedに移植)
// 制約内にあるときに誤差が増大する関数とする
struct RevRangeConS : Constraint{

	int _prohsize;

	// 配列サイズは適当
	vec2_t  _ConCenter[10];	// 制約の中心座標
	real_t	_r[10];			// 中心からの範囲

	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	RevRangeConS(Solver* solver, ID id, SVar* var0, SVar* var1,real_t _scale);

};

// 遊脚軌道についての評価
struct MidRangeConS : Constraint{

	int _prohsize;

	// 配列サイズは適当
	vec2_t  _Cen[10];	// 制約の中心座標
	real_t	_r[10];			// 中心からの範囲

	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	MidRangeConS(Solver* solver, ID id, SVar* var0, SVar* var1, SVar* var2, SVar* var3, real_t _scale);

};

// 動障害物についての評価
struct RevMoveConS : Constraint{

	int _movesize;

	// 配列サイズは適当
	vec2_t  _ConIni[10];	// 動制約のスタート地点
	real_t	_r[10];			// 中心からの範囲
	real_t  _vx[10];		// 障害物x軸方向速度
	real_t  _vy[10];		// 障害物y軸方向速度
	real_t  t;			// ステップ開始時の時刻

	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	RevMoveConS(Solver* solver, ID id, SVar* var0, SVar* var1, SVar* var2, real_t _scale);

};

}
