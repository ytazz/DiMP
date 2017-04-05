class  RevRangeConS;
class  MidRangeConS;
class  RevMoveConS;

	RevRangeConS*	con_proh_cop;		///< (add) step prohibited area 
	MidRangeConS*	con_proh_mid;		///< (add)
	RevMoveConS*	con_move_cop;		///< (add) moving constraint for cop

		real_t control_period ;			///<���ۂ̐������
		real_t calc_period ;			///<�v�Z��̎���

		real_t start_move_time;			///< �ŏ��ɏ����ʒu�܂ő���������Ă��鎞��
		real_t foot_place;				///< �d�S����̏�������ʒu�i���̍L���j
		vec2_t rfoot_place;				///< �d�S����̏����E����ʒu
		vec2_t lfoot_place;				///< �d�S����̏���������ʒu
		
		vec2_t proh_pos;				///< �ڒn�s���S
		real_t proh_r;					///< �ڒn�s�̈攼�a


		vec2_t initial_velocity;		///< �d�S�����x

		real_t max_swing_height;			///< �V�r�����̍ő�l

		bool out_csv;					///<���[�V�����f�[�^���o�͂��邩
		bool use_beginning;				///<������Ԃ�^���邩
		bool use_terminal_pos ;			///<�I�[������^���邩
		bool use_terminal_vel ;			///<�I�[���x��^���邩
		bool out_enuvo;					///<enuvo�̋��L�������ɏo�͂��邩
		bool online;					///<�I�����C�����䂷�邩
		bool use_initial_trajectory;	///<�����O����^���邩

		std::map<int, int>	phase;		///< walking phase at each step

		//���[�V�����f�[�^�쐬���Ɏg�p
		real_t alpha;					/// �����ʁ~alpha�����̎��� 
		real_t beta;					/// ���̒��S����̑��̏d�S�����~beta �� ���ۂ̑��̒���
		real_t real_height;				/// ���̍���

		Param();
	};
	/// �o�R�_
	struct Waypoint{
		int     k;
		vec2_t  pos;
		vec2_t  vel;
	};
	/// �O��
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
	float tm;	// �`�ʎ���
	clock_t start,test;

	Vec3f pos_right, pos_left;


// test add 11/19 (�����I�ɂ�Biped�ɈڐA)
// ������ɂ���Ƃ��Ɍ덷�����傷��֐��Ƃ���
struct RevRangeConS : Constraint{

	int _prohsize;

	// �z��T�C�Y�͓K��
	vec2_t  _ConCenter[10];	// ����̒��S���W
	real_t	_r[10];			// ���S����͈̔�

	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	RevRangeConS(Solver* solver, ID id, SVar* var0, SVar* var1,real_t _scale);

};

// �V�r�O���ɂ��Ă̕]��
struct MidRangeConS : Constraint{

	int _prohsize;

	// �z��T�C�Y�͓K��
	vec2_t  _Cen[10];	// ����̒��S���W
	real_t	_r[10];			// ���S����͈̔�

	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	MidRangeConS(Solver* solver, ID id, SVar* var0, SVar* var1, SVar* var2, SVar* var3, real_t _scale);

};

// ����Q���ɂ��Ă̕]��
struct RevMoveConS : Constraint{

	int _movesize;

	// �z��T�C�Y�͓K��
	vec2_t  _ConIni[10];	// ������̃X�^�[�g�n�_
	real_t	_r[10];			// ���S����͈̔�
	real_t  _vx[10];		// ��Q��x���������x
	real_t  _vy[10];		// ��Q��y���������x
	real_t  t;			// �X�e�b�v�J�n���̎���

	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	RevMoveConS(Solver* solver, ID id, SVar* var0, SVar* var1, SVar* var2, real_t _scale);

};

}
