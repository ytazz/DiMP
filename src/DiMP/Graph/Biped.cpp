#include <DiMP/Graph/Biped.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

#include<stdio.h>

namespace DiMP {
	;

	const real_t pi = M_PI;





	//-------------------------------------------------------------------------------------------------
	// BipedLIPKey

	BipedLIPKey::BipedLIPKey() {

	}

	//�ϐ����ǉ������֐�
	void BipedLIPKey::AddVar(Solver* solver) {
		BipedLIP* obj = (BipedLIP*)node;

		//���̈ʒu�E���x
		var_torso_pos_t = new V2Var(solver, ID(VarTag::BipedTorsoTP, node, tick, name + "_torso_tp"), node->graph->scale.pos_t);
		var_torso_pos_r = new SVar(solver, ID(VarTag::BipedTorsoRP, node, tick, name + "_torso_rp"), node->graph->scale.pos_r);
		var_torso_vel_t = new V2Var(solver, ID(VarTag::BipedTorsoTV, node, tick, name + "_torso_tv"), node->graph->scale.vel_t);
		var_torso_vel_r = new SVar(solver, ID(VarTag::BipedTorsoRV, node, tick, name + "_torso_rv"), node->graph->scale.vel_r);

		//�����ʒu
		var_foot_pos_t[0] = new V2Var(solver, ID(VarTag::BipedFootT, node, tick, name + "_foot_r_t"), node->graph->scale.pos_t);
		var_foot_pos_r[0] = new SVar(solver, ID(VarTag::BipedFootR, node, tick, name + "_foot_r_r"), node->graph->scale.pos_r);
		var_foot_pos_t[1] = new V2Var(solver, ID(VarTag::BipedFootT, node, tick, name + "_foot_l_t"), node->graph->scale.pos_t);
		var_foot_pos_r[1] = new SVar(solver, ID(VarTag::BipedFootR, node, tick, name + "_foot_l_r"), node->graph->scale.pos_r);

		//�d�S�ʒu�E���x
		var_com_pos = new V2Var(solver, ID(VarTag::BipedComP, node, tick, name + "_com_p"), node->graph->scale.pos_t);
		var_com_vel = new V2Var(solver, ID(VarTag::BipedComV, node, tick, name + "_com_v"), node->graph->scale.vel_t);

		//CoP�ʒu
		var_cop_pos = new V2Var(solver, ID(VarTag::BipedCop, node, tick, name + "_cop"), node->graph->scale.pos_t);

		//���s����
		var_duration = new SVar(solver, ID(VarTag::BipedDuration, node, tick, name + "_duration"), node->graph->scale.time);
		//����
		var_time = new SVar(solver, ID(VarTag::BipedTime, node, tick, name + "_time"), node->graph->scale.time);
	}

	//�S���������ǉ������֐�
	void BipedLIPKey::AddCon(Solver* solver) {
		BipedLIP* obj = (BipedLIP*)node;
		BipedLIPKey* nextObj = (BipedLIPKey*)next;

		if (next) {
			con_lip_p = new BipedLipConP(solver, name + "_lip_p", this, node->graph->scale.pos_t);
			con_lip_v = new BipedLipConV(solver, name + "_lip_v", this, node->graph->scale.vel_t);
		}

		con_foot_t[0] = new BipedFootConT(solver, name + "_foot_range_r_t", this, 0, node->graph->scale.pos_t);
		con_foot_r[0] = new BipedFootConR(solver, name + "_foot_range_r_r", this, 0, node->graph->scale.pos_r);
		con_foot_t[1] = new BipedFootConT(solver, name + "_foot_range_l_t", this, 1, node->graph->scale.pos_t);
		con_foot_r[1] = new BipedFootConR(solver, name + "_foot_range_l_r", this, 1, node->graph->scale.pos_r);

		int phase = obj->phase[tick->idx];
		int side;
		if (phase == BipedLIP::Phase::R || phase == BipedLIP::Phase::RL)
			side = 0;
		else side = 1;
		con_cop = new BipedCopCon(solver, name + "_cop", this, side, node->graph->scale.pos_t);

		if (next) {
			con_duration = new RangeConS(solver, ID(ConTag::BipedDuration, node, tick, name + "_duration"), var_duration, node->graph->scale.time);
			con_time = new BipedTimeCon(solver, name + "_time", this, node->graph->scale.time);
		}

		if (next) {
			con_foot_match_t[0] = new MatchConV2(solver, ID(ConTag::BipedFootMatchT, node, tick, name + "_foot_match_r_t"), var_foot_pos_t[0], nextObj->var_foot_pos_t[0], node->graph->scale.pos_t);
			con_foot_match_r[0] = new MatchConS(solver, ID(ConTag::BipedFootMatchR, node, tick, name + "_foot_match_r_r"), var_foot_pos_r[0], nextObj->var_foot_pos_r[0], node->graph->scale.pos_r);
			con_foot_match_t[1] = new MatchConV2(solver, ID(ConTag::BipedFootMatchT, node, tick, name + "_foot_match_l_t"), var_foot_pos_t[1], nextObj->var_foot_pos_t[1], node->graph->scale.pos_t);
			con_foot_match_r[1] = new MatchConS(solver, ID(ConTag::BipedFootMatchR, node, tick, name + "_foot_match_l_r"), var_foot_pos_r[1], nextObj->var_foot_pos_r[1], node->graph->scale.pos_r);
		}

		con_com_p = new BipedComConP(solver, name + "_com_p", this, node->graph->scale.pos_t);
		con_com_v = new BipedComConV(solver, name + "_com_v", this, node->graph->scale.vel_t);
	}

	void BipedLIPKey::Prepare() {
		BipedLIP* obj = (BipedLIP*)node;

		// ���s�����ϐ��̒l�������ɔ��f
		tick->time = var_time->val;

		if (!prev) {
			// �����������Œ�
			var_time->val = 0.0;
			var_time->locked = true;
		}

		if (next) {
			int phase = obj->phase[tick->idx];
			con_foot_match_t[0]->enabled = (phase != BipedLIP::Phase::L);
			con_foot_match_r[0]->enabled = (phase != BipedLIP::Phase::L);
			con_foot_match_t[1]->enabled = (phase != BipedLIP::Phase::R);
			con_foot_match_r[1]->enabled = (phase != BipedLIP::Phase::R);
		}
	}

	void BipedLIPKey::Draw(Render::Canvas* canvas, Render::Config* conf) {
		BipedLIP* obj = (BipedLIP*)node;

		Vec3f pcom, pcop, pf[2], pt;
		float thetaf[2];

		canvas->SetPointSize(5.0f); //�_�̑傫��
		canvas->SetLineWidth(1.0f); //���̑���

		// com
		int n, n1;
		real_t heightd;
		FILE* fp = fopen("Test.csv", "r");
		FILE* fp1 = fopen("Test1.csv", "r");
		fscanf(fp, "%d", &n);
		fscanf(fp1, "%d", &n1);
		if(n==0 && n1==0){
		 	heightd = ((BipedLIP*)node)->param.heightlow;
		}
	  else if(n==1 && n1==1){
		 	heightd = ((BipedLIP*)node)->param.heighthigh;
		 }
		 else{
			 heightd = ((BipedLIP*)node)->param.heightmiddle;
		 }
		fclose(fp);
		fclose(fp1);
		pcom.x = (float)var_com_pos->val.x;
		pcom.y = (float)var_com_pos->val.y;
		pcom.z = (float)heightd;//(float)((BipedLIP*)node)->param.heightCoM;
		canvas->Point(pcom); //�d�S�ʒu���v���b�g

							 // feet
		Vec2f cmin = obj->param.FootShapeMin; //CoPMinmum
		Vec2f cmax = obj->param.FootShapeMax; //CoPMax

		for (uint i = 0; i < 2; i++) {
			pf[i].x = (float)var_foot_pos_t[i]->val.x;
			pf[i].y = (float)var_foot_pos_t[i]->val.y;
			pf[i].z = 0.0f;
			canvas->Point(pf[i]);

			// foot print
			canvas->Push();
			thetaf[i] = (float)var_foot_pos_r[i]->val;
			canvas->Translate(pf[i].x, pf[i].y, pf[i].z);
			canvas->Rotate(thetaf[i], Vec3f(0.0f, 0.0f, 1.0f));
			canvas->Line(Vec3f(cmax.x, cmax.y, 0.0f), Vec3f(cmin.x, cmax.y, 0.0f));
			canvas->Line(Vec3f(cmin.x, cmax.y, 0.0f), Vec3f(cmin.x, cmin.y, 0.0f));
			canvas->Line(Vec3f(cmin.x, cmin.y, 0.0f), Vec3f(cmax.x, cmin.y, 0.0f));
			canvas->Line(Vec3f(cmax.x, cmin.y, 0.0f), Vec3f(cmax.x, cmax.y, 0.0f));
			canvas->Pop();
		}

		// torso
		pt = obj->TorsoPos(pcom, pf[0], pf[1]);
		canvas->Point(pt); //���̈ʒu���v���b�g

						   // lines connecting torso and feet
		canvas->Line(pt, pf[0]); //���̈ʒu�Ƒ����ʒu������
		canvas->Line(pt, pf[1]);

		// cop
		pcop.x = (float)var_cop_pos->val.x;
		pcop.y = (float)var_cop_pos->val.y;
		pcop.z = 0.0f;
		canvas->Point(pcop); //CoP�ʒu���v���b�g

	}

	//-------------------------------------------------------------------------------------------------
	// BipedLIP

	BipedLIP::Param::Param() {
		gravity = 9.8; //�d�͉����x
		heightCoM = 0.496; //�d�S����
		heightlow = 0.45;
		heighthigh = 0.51;
		//heightmiddle = (2*heightlow + heighthigh)/3;
		heightmiddle = (heightlow + heighthigh)/2;
		torsoMass = 4.432*0.8; //���̎���
		footMass = (4.432 - torsoMass) / 2; //������
		//swingProfile = SwingProfile::Wedge;  //�V�r�O���̎���
		//swingProfile = SwingProfile::Cycloid;
		swingProfile = SwingProfile::Heel_Toe;
		thetaHeel = 20 * pi/180;
		thetaToe = 20 * pi/180;
		swingHeight[0] = 0.1; //0:�V�r�̍ő卂��
		swingHeight[1] = 0.1; //1:�ڒn�O�̍ŏ�����(Wedge only)
		durationMin[Phase::R] = 0.1; //�Б��x����Minimum
		durationMax[Phase::R] = 0.8; //�Б��x����Max
		durationMin[Phase::L] = 0.1;
		durationMax[Phase::L] = 0.8;
		durationMin[Phase::RL] = 0.1; //�����x����Minimum
		durationMax[Phase::RL] = 0.2; //�����x����Max
		durationMin[Phase::LR] = 0.1;
		durationMax[Phase::LR] = 0.2;

		//�����\��CoM��CoP�̑��΋���
		footPosMin[0] = vec2_t(-0.2, -0.14);
		footPosMax[0] = vec2_t(0.2, -0.07);
		footPosMin[1] = vec2_t(-0.2, 0.07);
		footPosMax[1] = vec2_t(0.2, 0.14);
		footOriMin[0] = -Rad(15.0);
		footOriMax[0] = Rad(15.0);
		footOriMin[1] = -Rad(15.0);
		footOriMax[1] = Rad(15.0);

		FootShapeMin = vec2_t(-0.0853/*-0.02667*/, -0.05); //FootMinmum
		FootShapeMax = vec2_t(0.0853, 0.05); //FootMax
		copPosMin = vec2_t(-0.0853, 0.0); //CoPMinmum
		copPosMax = vec2_t(0.0853, 0.0); //CoPMax

		// FootShapeMin = vec2_t(-0.1/*-0.02667*/, -0.05); //FootMinmum
		// FootShapeMax = vec2_t(0.1, 0.05); //FootMax
		// copPosMin = vec2_t(-0.1, 0.0); //CoPMinmum
		// copPosMax = vec2_t(0.1, 0.0); //CoPMax

		// FootShapeMin = vec2_t(-0.05/*-0.02667*/, -0.05); //FootMinmum
		// FootShapeMax = vec2_t(0.09, 0.05); //FootMax
		// copPosMin = vec2_t(-0.05, 0.0); //CoPMinmum
		// copPosMax = vec2_t(0.09, 0.0); //CoPMax

		angAccMax = 0.3;
		turnMax = 0.5;

		// int n;
		// FILE* fp = fopen("Test.csv", "r");
		// fscanf(fp, "%d", &n);
		//
		// if(n==0){
		// 	heightCoM = 0.4;
		// }
		// else{
		// 	heightCoM = 0.5;
		// }
		//
		// fprintf(fp1, "%lf, \n", heightCoM);
		// fclose(fp);
	}

	//-------------------------------------------------------------------------------------------------
	/// �o�R�_
	//BipedLIP�N���X�̍\����Waypoint�̃����o�֐�Waypoint()
	BipedLIP::Waypoint::Waypoint() {
		k = 0;
		time = 0.0;
		torso_pos_t = vec2_t();
		torso_pos_r = 0.0;
		torso_vel_t = vec2_t();
		torso_vel_r = 0.0;
		foot_pos_t[0] = vec2_t();
		foot_pos_r[0] = 0.0;
		foot_pos_t[1] = vec2_t();
		foot_pos_r[1] = 0.0;

		fix_torso_pos_t = false;
		fix_torso_pos_r = false;
		fix_torso_vel_t = false;
		fix_torso_vel_r = false;
		fix_foot_pos_t[0] = false;
		fix_foot_pos_r[0] = false;
		fix_foot_pos_t[1] = false;
		fix_foot_pos_r[1] = false;
	}

	//-------------------------------------------------------------------------------------------------
	//BipedLIP�N���X�̍\����TrajPoint�̃����o�֐�Trajpoint()
	BipedLIP::TrajPoint::TrajPoint() {
		t = 0.0;
		torso_pos_t = vec3_t();
		torso_pos_r = 0.0;
		foot_pos_t[0] = vec3_t();
		foot_pos_r[0] = quat_t();
		foot_pos_t[1] = vec3_t();
		foot_pos_r[1] = quat_t();
	}

	//-------------------------------------------------------------------------------------------------

	BipedLIP::BipedLIP(Graph* g, string n) :TrajectoryNode(g, n) {
		type = Type::Object;
		graph->bipeds.Add(this);
	}

	BipedLIP::~BipedLIP() {
		graph->bipeds.Remove(this);
	}

	void BipedLIP::Init() {
		TrajectoryNode::Init();

		int n, n1;
		real_t heightd;
		FILE* fp = fopen("Test.csv", "r");
		FILE* fp1 = fopen("Test1.csv", "r");
		fscanf(fp, "%d", &n);
		fscanf(fp1, "%d", &n1);
		if(n==0 && n1==0){
		 	heightd = param.heightlow;
		}
	  else if(n==1 && n1==1){
		 	heightd = param.heighthigh;
		 }
		 else{
			 heightd = param.heightmiddle;
		 }
		fclose(fp);
		fclose(fp1);

		param.T = sqrt(heightd/*param.heightCoM*/ / param.gravity);//���萔T=��h/g

		real_t durationAve[Phase::Num];
		for (int i = 0; i < Phase::Num; i++)
			durationAve[i] = (param.durationMin[i] + param.durationMax[i]) / 2.0;

		real_t t = 0.0;
		for (uint k = 0; k < graph->ticks.size(); k++) {
			BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[k]);

			key->var_time->val = t;

			// �����̏����l�͉����Ə����̒��Ԓl
			if (key->next) {
				key->var_duration->val = durationAve[phase[k]];
				key->con_duration->_min = param.durationMin[phase[k]];
				key->con_duration->_max = param.durationMax[phase[k]];
			}

			/*
			//�X�e�b�v�n�_�ƏI�_�̊p�����x����
			key->con_com_r[0]->_min = -param.angAccMax;
			key->con_com_r[0]->_max =  param.angAccMax;
			key->con_com_r[1]->_min = -param.angAccMax;
			key->con_com_r[1]->_max =  param.angAccMax;
			*/

			//�����\��CoM��CoP�̑��΋���
			for (int i = 0; i < 2; i++) {
				key->con_foot_t[i]->_min = param.footPosMin[i];
				key->con_foot_t[i]->_max = param.footPosMax[i];
				key->con_foot_r[i]->_min = param.footOriMin[i];
				key->con_foot_r[i]->_max = param.footOriMax[i];
			}

			key->con_cop->_min = param.copPosMin;
			key->con_cop->_max = param.copPosMax;

			t += durationAve[phase[k]];
		}

		// �d�S�ʒu�C�d�S���x�C���n�ʒu�̏����l���o�R�_�����ԃX�v���C���Ȑ�(���炩�ȕ��ԋȐ�)�ŗ^����
		Curve2d curve_torso_t, curve_foot_t[2];
		Curved  curve_torso_r, curve_foot_r[2];

		curve_torso_t.SetType(Interpolate::Cubic);
		curve_torso_r.SetType(Interpolate::Cubic);
		curve_foot_t[0].SetType(Interpolate::Cubic);
		curve_foot_r[0].SetType(Interpolate::Cubic);
		curve_foot_t[1].SetType(Interpolate::Cubic);
		curve_foot_r[1].SetType(Interpolate::Cubic);

		for (uint i = 0; i < waypoints.size(); i++) {
			Waypoint& wp = waypoints[i];
			BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[wp.k]);
			real_t t = key->var_time->val;

			curve_torso_t.AddPoint(t);
			curve_torso_t.SetPos(i, wp.torso_pos_t);
			curve_torso_t.SetVel(i, wp.torso_vel_t);

			curve_torso_r.AddPoint(t);
			curve_torso_r.SetPos(i, wp.torso_pos_r);
			curve_torso_r.SetVel(i, wp.torso_vel_r);

			for (uint j = 0; j < 2; j++) {
				curve_foot_t[j].AddPoint(t);
				curve_foot_t[j].SetPos(i, wp.foot_pos_t[j]);
				curve_foot_t[j].SetVel(i, vec2_t());

				curve_foot_r[j].AddPoint(t);
				curve_foot_r[j].SetPos(i, wp.foot_pos_r[j]);
				curve_foot_r[j].SetVel(i, 0.0);
			}
		}

		for (uint k = 0; k < graph->ticks.size(); k++) {
			BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[k]);
			real_t t = key->var_time->val;

			key->var_torso_pos_t->val = curve_torso_t.CalcPos(t);
			key->var_torso_pos_r->val = curve_torso_r.CalcPos(t);

			key->var_torso_vel_t->val = curve_torso_t.CalcVel(t);
			key->var_torso_vel_r->val = curve_torso_r.CalcVel(t);

			for (uint j = 0; j < 2; j++) {
				key->var_foot_pos_t[j]->val = curve_foot_t[j].CalcPos(t);
				key->var_foot_pos_r[j]->val = curve_foot_r[j].CalcPos(t);
			}

			real_t mt = param.torsoMass;
			real_t mf = param.footMass;
			key->var_com_pos->val = (mt * key->var_torso_pos_t->val + mf * (key->var_foot_pos_t[0]->val + key->var_foot_pos_t[1]->val)) / (mt + 2.0*mf);
			key->var_com_vel->val = (mt * key->var_torso_vel_t->val) / (mt + 2.0*mf);

			if (phase[k] == Phase::R || phase[k] == Phase::RL)
				key->var_cop_pos->val = key->var_foot_pos_t[0]->val;
			else key->var_cop_pos->val = key->var_foot_pos_t[1]->val;
		}

		// �o�R�_���̕ϐ����Œ�
		for (uint i = 0; i < waypoints.size(); i++) {
			Waypoint& wp = waypoints[i];
			BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[wp.k]);

			key->var_torso_pos_t->locked = wp.fix_torso_pos_t;
			key->var_torso_pos_r->locked = wp.fix_torso_pos_r;
			key->var_torso_vel_t->locked = wp.fix_torso_vel_t;
			key->var_torso_vel_r->locked = wp.fix_torso_vel_r;

			for (uint j = 0; j < 2; j++) {
				key->var_foot_pos_t[j]->locked = wp.fix_foot_pos_t[j];
				key->var_foot_pos_r[j]->locked = wp.fix_foot_pos_r[j];
			}
		}
	}

	void BipedLIP::Prepare() {
		TrajectoryNode::Prepare();
		trajReady = false;
	}

	int BipedLIP::Phase(real_t t) {
		BipedLIPKey* key = (BipedLIPKey*)traj.GetSegment(t).first;
		return phase[key->tick->idx];
	}

	//�d�S�ʒu
	vec3_t BipedLIP::ComPos(real_t t) {
		BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
		BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

		vec2_t pt;

		if (key1) {
			real_t T = param.T;
			real_t dt = t - key0->var_time->val;
			real_t tau = key0->var_duration->val;
			vec2_t p0 = key0->var_com_pos->val;
			vec2_t v0 = key0->var_com_vel->val;
			vec2_t c0 = key0->var_cop_pos->val;
			vec2_t c1 = key1->var_cop_pos->val;

			pt = c0 + (c1 - c0)*(dt / tau) + (p0 - c0)*cosh(dt / T) + T * (v0 - (c1 - c0) / tau)*sinh(dt / T); //��(39)
		}
		else {
			pt = key0->var_com_pos->val;
		}
		int n, n1;
		real_t heightd;
		FILE* fp = fopen("Test.csv", "r");
		FILE* fp1 = fopen("Test1.csv", "r");
		fscanf(fp, "%d", &n);
		fscanf(fp1, "%d", &n1);
		if(n==0 && n1==0){
		 	heightd = param.heightlow;
		}
	  else if(n==1 && n1==1){
		 	heightd = param.heighthigh;
		 }
		 else{
			 heightd = param.heightmiddle;
		 }
		fclose(fp);
		fclose(fp1);

		return vec3_t(pt.x/*+0.04*/, pt.y, heightd/*param.heightCoM*/);
	}

	//�d�S���x
	vec3_t BipedLIP::ComVel(real_t t) {
		BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
		BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

		vec2_t vt;

		if (key1) {
			real_t T = param.T;
			real_t dt = t - key0->var_time->val;
			real_t tau = key0->var_duration->val;
			vec2_t p0 = key0->var_com_pos->val;
			vec2_t v0 = key0->var_com_vel->val;
			vec2_t c0 = key0->var_cop_pos->val;
			vec2_t c1 = key1->var_cop_pos->val;

			vt = (c1 - c0) / tau + (1 / T)*(p0 - c0)*sinh(dt / T) + (v0 - (c1 - c0) / tau)*cosh(dt / T); //��(40)
		}
		else {
			vt = key0->var_com_vel->val;
		}

		return vec3_t(vt.x, vt.y, 0.0);
	}

	//�d�S�����x
	vec3_t BipedLIP::ComAcc(real_t t) {
		BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
		BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

		vec2_t at;

		if (key1) {
			real_t T = param.T;
			real_t T2 = T * T;
			real_t dt = t - key0->var_time->val;
			real_t tau = key0->var_duration->val;
			vec2_t p0 = key0->var_com_pos->val;
			vec2_t v0 = key0->var_com_vel->val;
			vec2_t c0 = key0->var_cop_pos->val;
			vec2_t c1 = key1->var_cop_pos->val;

			vec2_t at = (1 / T2)*(p0 - c0)*cosh(dt / T) + (1 / T)*(v0 - (c1 - c0) / tau)*sinh(dt / T); //��(41)
		}
		else {
			at.clear();
		}

		return vec3_t(at.x, at.y, 0.0);
	}

	//���̊p�x
	real_t BipedLIP::TorsoOri(real_t t) {
		BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
		BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

		real_t o;

		if (key1) {
			real_t t0 = key0->var_time->val;
			real_t t1 = key1->var_time->val;

			o = InterpolatePos(
				t,
				t0,
				key0->var_torso_pos_r->val,
				key0->var_torso_vel_r->val,
				t1,
				key1->var_torso_pos_r->val,
				key1->var_torso_vel_r->val,
				Interpolate::Cubic);
		}
		else {
			o = key0->var_torso_pos_r->val;
		}

		return o;
	}

	//���̊p���x
	real_t BipedLIP::TorsoAngVel(real_t t) {
		BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
		BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

		real_t r;

		if (key1) {
			real_t t0 = key0->var_time->val;
			real_t t1 = key1->var_time->val;

			r = InterpolateVel(
				t,
				t0,
				key0->var_torso_pos_r->val,
				key0->var_torso_vel_r->val,
				t1,
				key1->var_torso_pos_r->val,
				key1->var_torso_vel_r->val,
				Interpolate::Cubic);
		}
		else {
			r = key0->var_torso_vel_r->val;
		}

		return r;
	}

	//���̊p���x
	real_t BipedLIP::TorsoAngAcc(real_t t) {
		BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
		BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

		real_t a;

		if (key1) {
			real_t t0 = key0->var_time->val;
			real_t t1 = key1->var_time->val;

			a = InterpolateAcc(
				t,
				t0,
				key0->var_torso_pos_r->val,
				key0->var_torso_vel_r->val,
				t1,
				key1->var_torso_pos_r->val,
				key1->var_torso_vel_r->val,
				Interpolate::Cubic);
		}
		else {
			a = 0.0;
		}

		return a;
	}


	//�����ʒu
	vec3_t BipedLIP::FootPos(real_t t, int side) {
		BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
		BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;
		real_t       t0 = key0->tick->time;
		real_t       t1 = key1->tick->time;
		real_t       h = t1 - t0;
		real_t       hhalf = h / 2.0;
    real_t       hspan = 0.3*h;
		real_t       t00 = t0 + hspan;
		real_t       t11 = t1 - hspan;
		real_t       td = (t0 + t1)/10;
		real_t       tdd = (t0 + t1)*29/30;

		vec2_t pt;
		real_t z = 0.0;
		real_t s = 0.0;
		real_t theta=0.0;
		real_t theta0 = 30*pi/180;
		real_t theta1 = -30*pi/180;
		// real_t l1 = 0.11;
		// real_t l2 = 0.11;
		//real_t thetaop = 10*pi/180;
		real_t thetatoe = 10*pi/180;
		real_t thetaheel = 10*pi/180;
		real_t theta_ds;
		int thetad;
		real_t px;

    real_t l = 0.16;
		real_t l0 = 0.08;
		real_t l1 = 0.08;
		real_t L = 0.20;
		//real_t r = 0.025;
		//real_t r = 0.02/thetaop;
		//real_t r0 = (L/2-l0)/thetatoe;
		//real_t r1 = (L/2-l1)/thetaheel;
		real_t r0 = 0.03;
		real_t r1 = 0.03;

		// real_t thetaps = 10*pi/180;
		// real_t l2 = 0.08;
		// real_t r = (L/2-l0)/thetaps;

    //foot_shape_diffferent
		// real_t l0 = 0.02;
		// real_t l1 = 0.08;
		// real_t l2 = 0.02;
		// real_t L = 0.20;
		// //real_t r = 0.025;
		// //real_t r = 0.02/thetaop;
		// real_t r0 = (L/2-l0-l2)/theta0;
		// real_t r1 = (L/2-l1)/thetaheel;

		int n;
		real_t dev = 0.0;



		if (key1) {
			real_t dt = t - key0->var_time->val;
			real_t tau = key0->var_duration->val;
			vec2_t p0 = key0->var_foot_pos_t[side]->val;
			vec2_t p1 = key1->var_foot_pos_t[side]->val;
			vec2_t c0 = key0->var_cop_pos->val;
			vec2_t c1 = key1->var_cop_pos->val;

			// �P�r�x�����̗V�r
			int ph = phase[key0->tick->idx];


			// if ((ph == Phase::R && side == 1) || (ph == Phase::L && side == 0)) { //(�E���Б��x���� or �����Б��x����)
			// 	if (param.swingProfile == SwingProfile::Wedge) {
			// 		if (t < t0 + hhalf) {
			// 			z = param.swingHeight[0];
			// 		}
			// 		else {
			// 			real_t a = (t - (t0 + hhalf)) / hhalf;
			// 			z = (1 - a)*param.swingHeight[0] + a * param.swingHeight[1];
			// 		}
			//
			// 		s = (t - t0) / h;//�����ɂ����鎞��t�̊���
			// 	}
			//
			// 	if (param.swingProfile == SwingProfile::Cycloid) {//���E�ܐ��̗V�r�O��
			// 		real_t _2pi = 2.0*M_PI;
			// 		real_t tau = (t - t0) / h;
			//
			// 		s = (tau - sin(_2pi*tau) / _2pi);
			// 		z = (param.swingHeight[0] / 2.0)*(1 - cos(_2pi*tau));
			// 	}

			// 	if (param.swingProfile == SwingProfile::Heel_Toe){
			// 		real_t _2pi = 2.0*M_PI;
			// 		real_t tau = (t - t0) / h;
			// 		real_t tau0 = (t - t0) / (t00 - t0);
			// 		real_t tau1 = (t - t11) / (t1 - t11);
			// 		real_t tau2 = (t - t00) / (t11 - t00);
			// 		real_t p00x = p0.x + l1*(1 - cos(theta0));
			// 		real_t p11x = p1.x - l2*(1 - cos(theta1));
      //     real_t p00z = l1*sin(theta0);
			// 		real_t p11z = l2*sin(-theta1);
			// 		real_t a = sqrt((p11x - p00x)*(p11x - p00x) + (p11z - p00z)*(p11z - p00z))/(2*pi);
			// 		real_t alpha = atan((p11z-p00z)/(p11x-p00x));
			// 		real_t d = p1.x - p0.x;
			//
			// 		if ((t >= t0) && (t <= t00)){
			// 			theta = tau0 * theta0;
			// 			s = l1 * (1-cos(theta))/d;
			// 			z = l1 * sin(theta);
			// 		}
			// 		else if ((t >= t11) && (t <= t1)){
			// 			theta = (1 - tau1)*theta1;
			// 			s = 1 - l2*(1-cos(theta))/d;
			// 			z = l2 * sin(-theta);
			// 		}
			// 		else {
			// 			real_t thetadash = 2*pi*tau2;
			// 			real_t px = p00x + a*((thetadash-sin(thetadash))*cos(alpha)-(1-cos(thetadash))*sin(alpha));
			// 			s = (px - p0.x)/d;
			// 			z = p00z + a*((1-cos(thetadash))*cos(alpha)+(thetadash-sin(thetadash))*sin(alpha))/2;
			// 		}
			// 	}
			// }

      //kataoka saigenn
			// if (param.swingProfile == SwingProfile::Heel_Toe){
			//   real_t _2pi = 2.0*M_PI;
			//   real_t tau = (t - t0) / h;
			//   real_t tau0 = (t - t0) / (t00 - t0);
			//   real_t tau1 = (t - t11) / (t1 - t11);
			//   real_t tau2 = (t - t00) / (t11 - t00);
			//   real_t p00x = p0.x + l1*(1 - cos(theta0));
			//   real_t p11x = p1.x - l2*(1 - cos(theta1));
			//   real_t p00z = l1*sin(theta0);
			//   real_t p11z = l2*sin(-theta1);
			//   real_t a = sqrt((p11x - p00x)*(p11x - p00x) + (p11z - p00z)*(p11z - p00z))/(2*pi);
			//   real_t alpha = atan((p11z-p00z)/(p11x-p00x));
			//   real_t d = p1.x - p0.x;
			//
			//   if ((ph == Phase::R && side == 1) || (ph == Phase::L && side == 0)){
			//     real_t thetadash = 2*pi*tau;
			//     real_t px = p00x + a*((thetadash-sin(thetadash))*cos(alpha)-(1-cos(thetadash))*sin(alpha));
			//     s = (px - p0.x)/d;
			//     z = p00z + a*((1-cos(thetadash))*cos(alpha)+(thetadash-sin(thetadash))*sin(alpha))/3;
			//   }
			//   else if ((ph == Phase::RL && side == 0) || (ph == Phase::LR && side == 1)){
			//     if(t<= t11){
			//       z = 0;
			//       s = 1;
			//     }
			//     else{
			//       theta = tau1 * theta0;
			//       s = 1 + l1 * (1-cos(theta))/d;
			//       z = l1 * sin(theta);
			//     }
			//   }
			//   else if ((ph == Phase::RL && side == 1) || (ph == Phase::LR && side == 0)) { //�ܐ您�낵
			//     if (t <= t00) {
			//       theta = (1 - tau0)*theta1;
			//       s = 1 - l2*(1-cos(theta))/d;
			//       z = l2 * sin(-theta);
			//     }
			//     else {
			//       z = 0;
			//       s = 1;
			//     }
			//   }
			//   else{
			//     z = 0;
			//     s = 1;
			//   }


		//kataoka_saigenn_new_style
		// if (param.swingProfile == SwingProfile::Heel_Toe){
		//   real_t _2pi = 2.0*M_PI;
		//   real_t tau = (t - t0) / h;
		//   real_t tau0 = (t - t0) / (t00 - t0);
		//   real_t tau1 = (t - t11) / (t1 - t11);
		//   real_t tau2 = (t - t00) / (t11 - t00);
		// 	real_t p00x = p0.x + r*(theta0 - sin(theta0)) + l1*(1 - cos(theta0));
 		// 	real_t p11x = p1.x - (r*(-theta1 - sin(-theta1)) + l2*(1 - cos(theta1)));
 		// 	real_t p00z = r*(1 - cos(theta0)) + l1*sin(theta0);
 		// 	real_t p11z = r*(1 - cos(theta1)) + l2*sin(-theta1);
		//   real_t a = sqrt((p11x - p00x)*(p11x - p00x) + (p11z - p00z)*(p11z - p00z))/(2*pi);
		//   real_t alpha = atan((p11z-p00z)/(p11x-p00x));
		//   real_t d = p1.x - p0.x;
		//
		//   if ((ph == Phase::R && side == 1) || (ph == Phase::L && side == 0)){
		//     real_t thetadash = 2*pi*tau;
		//     real_t px = p00x + a*((thetadash-sin(thetadash))*cos(alpha)-(1-cos(thetadash))*sin(alpha));
		//     s = (px - p0.x)/d;
		//     z = p00z + a*((1-cos(thetadash))*cos(alpha)+(thetadash-sin(thetadash))*sin(alpha))/3;
		//   }
		//   else if ((ph == Phase::RL && side == 0) || (ph == Phase::LR && side == 1)){
		//     if(t<= t11){
		//       z = 0;
		//       s = 1;
		//     }
		//     else{
		//       theta = tau1 * theta0;
		// 			s = 1 + (r*(theta - sin(theta)) + l1*(1-cos(theta)))/d;
		// 			z = r*(1 - cos(theta)) + l1*sin(theta);
		//     }
		//   }
		//   else if ((ph == Phase::RL && side == 1) || (ph == Phase::LR && side == 0)) { //�ܐ您�낵
		//     if (t <= t00) {
		//       theta = -(1 - tau0)*theta1;
		// 			s = 1 - (r*(theta - sin(theta)) + l2*(1-cos(theta)))/d;
		//  			z = r*(1 - cos(theta)) + l2*sin(theta);
		//     }
		//     else {
		//       z = 0;
		//       s = 1;
		//     }
		//   }
		//   else{
		//     z = 0;
		//     s = 1;
		//   }


    //new
		// if (param.swingProfile == SwingProfile::Heel_Toe){
		// 	real_t _2pi = 2.0*M_PI;
		// 	real_t tau = (t - t0) / h;
		// 	real_t tau1 = (t - t0) / (td - t0);
		// 	real_t tau2 = (t - tdd) / (t1 - tdd);
		// 	real_t p00x = p0.x + l1*(1 - cos(theta0));
		// 	real_t p11x = p1.x - l2*(1 - cos(theta1));
		// 	real_t p00z = l1*sin(theta0);
		// 	real_t p11z = l2*sin(-theta1);
		// 	real_t a = sqrt((p11x - p00x)*(p11x - p00x) + (p11z - p00z)*(p11z - p00z))/(2*pi);
		// 	real_t alpha = atan((p11z-p00z)/(p11x-p00x));
		// 	real_t d = p1.x - p0.x;
		//
		// 	if ((ph == Phase::R && side == 1) || (ph == Phase::L && side == 0)){
		// 		real_t thetadash = 2*pi*tau;
		// 		real_t px = p00x + a*((thetadash-sin(thetadash))*cos(alpha)-(1-cos(thetadash))*sin(alpha));
		// 		s = (px - p0.x)/d;
		// 		z = p00z + a*((1-cos(thetadash))*cos(alpha)+(thetadash-sin(thetadash))*sin(alpha))/3;
		// 	}
		// 	else if ((ph == Phase::RL && side == 0) || (ph == Phase::LR && side == 1)){
		// 		theta = thetaps + tau*(theta0-thetaps);
		// 		s = 1 + l1*(1-cos(theta))/d;
		// 		z = l1*sin(theta);
		// 	}
		// 	else if ((ph == Phase::RL && side == 1) || (ph == Phase::LR && side == 0)) { //�ܐ您�낵
		// 		if(t<= td){
		// 			theta = (1-tau1)*theta1;
		// 			s = 1 - l2*(1-cos(theta))/d;
		// 			z = l2*sin(-theta);
		// 		}
		// 		else{
		// 			s = 1;
		// 			z = 0;
		// 		}
		// 	}
		// 	else{
		// 		if(t<= tdd){
		// 			s = 1;
		// 			z = 0;
		// 		}
		// 		else{
		// 			theta = tau2*thetaps;
		// 			s = 1 + l1*(1-cos(theta))/d;
		// 			z = l1*sin(theta);
		// 		}
		// }

	  //new_style
		 // if (param.swingProfile == SwingProfile::Heel_Toe){
 			// real_t _2pi = 2.0*M_PI;
 			// real_t tau = (t - t0) / h;
 			// real_t tau1 = (t - t0) / (td - t0);
 			// real_t tau2 = (t - tdd) / (t1 - tdd);
 			// real_t p00x = p0.x + r*(theta0 - sin(theta0)) + l1*(1 - cos(theta0));
 			// real_t p11x = p1.x - (r*(-theta1 - sin(-theta1)) + l2*(1 - cos(theta1)));
 			// real_t p00z = r*(1 - cos(theta0)) + l1*sin(theta0);
 			// real_t p11z = r*(1 - cos(theta1)) + l2*sin(-theta1);
 			// real_t a = sqrt((p11x - p00x)*(p11x - p00x) + (p11z - p00z)*(p11z - p00z))/(2*pi);
 			// real_t alpha = atan((p11z-p00z)/(p11x-p00x));
 			// real_t d = p1.x - p0.x;
		 //
 			// if ((ph == Phase::R && side == 1) || (ph == Phase::L && side == 0)){
 			// 	real_t thetadash = 2*pi*tau;
 			// 	real_t px = p00x + a*((thetadash-sin(thetadash))*cos(alpha)-(1-cos(thetadash))*sin(alpha));
 			// 	s = (px - p0.x)/d;
 			// 	z = p00z + a*((1-cos(thetadash))*cos(alpha)+(thetadash-sin(thetadash))*sin(alpha))/3;
 			// }
 			// else if ((ph == Phase::RL && side == 0) || (ph == Phase::LR && side == 1)){
 			// 	theta = thetaps + tau*(theta0-thetaps);
 			// 	s = 1 + (r*(theta - sin(theta)) + l1*(1-cos(theta)))/d;
 			// 	z = r*(1 - cos(theta)) + l1*sin(theta);
 			// }
 			// else if ((ph == Phase::RL && side == 1) || (ph == Phase::LR && side == 0)) { //�ܐ您�낵
 			// 	if(t<= td){
 			// 		theta = -(1-tau1)*theta1;
 			// 		s = 1 - (r*(theta - sin(theta)) + l2*(1-cos(theta)))/d;
 			// 		z = r*(1 - cos(theta)) + l2*sin(theta);
 			// 	}
 			// 	else{
 			// 		s = 1;
 			// 		z = 0;
 			// 	}
 			// }
 			// else{
 			// 	if(t<= tdd){
 			// 		s = 1;
 			// 		z = 0;
 			// 	}
 			// 	else{
 			// 		theta = tau2*thetaps;
 			// 		s = 1 + (r*(theta - sin(theta)) + l1*(1-cos(theta)))/d;
 			// 		z = r*(1 - cos(theta)) + l1*sin(theta);
 			// 	}
 			// }

		//gait_change
		// if (param.swingProfile == SwingProfile::Heel_Toe){
		// 	real_t _2pi = 2.0*M_PI;
		// 	real_t tau = (t - t0) / h;
		// 	real_t tau1 = (t - t0) / (td - t0);
		// 	real_t tau2 = (t - tdd) / (t1 - tdd);
		// 	real_t p00x = p0.x + r*(theta0 - sin(theta0)) + l/2*(1 - cos(theta0));
		// 	real_t p11x = p1.x - (r*(-theta1 - sin(-theta1)) + l/2*(1 - cos(theta1)));
		// 	real_t p00z = r*(1 - cos(theta0)) + l/2*sin(theta0);
		// 	real_t p11z = r*(1 - cos(theta1)) + l/2*sin(-theta1);
		// 	real_t d = p1.x - p0.x;
		// 	real_t c = c0.x + (c1.x - c0.x)*tau;
		// 	real_t thetadash = 2*pi*tau;
		//
		// 	if((c0.x - p1.x) > l/2){
		// 		theta_ds = (c0.x - p1.x - l/2)/r;
		// 	}
		// 	else{
		// 		theta_ds = 0;
		// 	}
		//
		// 	if(ph == Phase::L && side == 0){
		//     FILE* fp = fopen("Test.csv", "r");
		//     fscanf(fp, "%lf", &thetad);
		//
		//     if(thetad == 0){
		//       //real_t alpha = atan(p11z/(p11x-p0.x));
		//       real_t a = d/(2*pi);
		//       px = p0.x + a*(thetadash-sin(thetadash));
		//       z = a*((1-cos(thetadash)))/2;
		//     }
		//     else{
		//       real_t a = (p11x - p00x)/(2*pi);
		//       px = p00x + a*(thetadash-sin(thetadash));
		//       z = p00z + a*(1-cos(thetadash))/3;
		//     }
		//     s = (px - p0.x)/d;
		//
		//     fclose(fp);
		//   }
		//   else if (ph == Phase::RL && side == 0){
		//     theta = theta_ds;
		//     s = 1 + (r*(theta - sin(theta)) + l/2*(1-cos(theta)))/d;
		//     z = r*(1 - cos(theta)) + l/2*sin(theta);
		//
		//     FILE* fp = fopen("Test.csv", "w");
		//     fprintf(fp, "%3.4lf", theta);
		//     fclose(fp);
		//
		//   }
		//   else if (ph == Phase::LR && side == 0) { //�ܐ您�낵
		//     FILE* fp = fopen("Test.csv", "r");
		//     fscanf(fp, "%lf", &thetad);
		//     if(thetad == 0){
		//       theta = 0;
		//     }
		//     else{
		//       theta = (p1.x - c1.x -l/2)/r;
		//     }
		//     s = 1 - (r*(theta - sin(theta)) + l/2*(1-cos(theta)))/d;
		//     z = r*(1 - cos(theta)) + l/2*sin(theta);
		//     fclose(fp);
		//   }
		//   else if (ph == Phase::R && side == 0){
		//     FILE* fp = fopen("Test.csv", "r");
		//     fscanf(fp, "%lf", &thetad);
		//
		//     if(((c0.x - p1.x) < -l/2) && ((c1.x - p1.x) > l/2)){
		//       if(c < (p1.x - l/2)){
		//         if(thetad==0){
		//           s = 1;
		//           z = 0;
		//         }
		//         else{
		//           theta = (p1.x - c - l/2)/r;
		//           s = 1 - (r*(theta - sin(theta)) + l/2*(1-cos(theta)))/d;
		//           z = r*(1 - cos(theta)) + l/2*sin(theta);
		//         }
		//       }
		//       else if(c > (p1.x + l/2)){
		//         theta = (c - p1.x - l/2)/r;
		//         s = 1 + (r*(theta - sin(theta)) + l/2*(1-cos(theta)))/d;
		//         z = r*(1 - cos(theta)) + l/2*sin(theta);
		//       }
		//       else{
		//         s = 1;
		//         z = 0;
		//       }
		//     }
		//     else{
		//       if(c < (p1.x - l/2)){
		//         if(thetad==0){
		//           s = 1;
		//           z = 0;
		//         }
		//         else{
		//           theta = (p1.x - c - l/2)/r;
		//           s = 1 - (r*(theta - sin(theta)) + l/2*(1-cos(theta)))/d;
		//           z = r*(1 - cos(theta)) + l/2*sin(theta);
		//         }
		//       }
		//       else{
		//         s = 1;
		//         z = 0;
		//       }
		//      }
		//
		//      fclose(fp);
		//
		//   }
		//
		//
		//
		//
		//   else if (ph == Phase::R && side == 1){
		//     FILE* fp1 = fopen("Test1.csv", "r");
		//     fscanf(fp1, "%lf", &thetad);
		//
		//     if(thetad == 0){
		//       //real_t alpha = atan(p11z/(p11x-p0.x));
		//       real_t a = d/(2*pi);
		//       px = p0.x + a*(thetadash-sin(thetadash));
		//       z = a*((1-cos(thetadash)))/2;
		//     }
		//     else{
		//       real_t a = (p11x - p00x)/(2*pi);
		//       px = p00x + a*(thetadash-sin(thetadash));
		//       z = p00z + a*(1-cos(thetadash))/3;
		//     }
		//     s = (px - p0.x)/d;
		//     fclose(fp1);
		//   }
		//   else if (ph == Phase::LR && side == 1){
		//     theta = theta_ds;
		//     s = 1 + (r*(theta - sin(theta)) + l/2*(1-cos(theta)))/d;
		//     z = r*(1 - cos(theta)) + l/2*sin(theta);
		//
		//     FILE* fp1 = fopen("Test1.csv", "w");
		//     fprintf(fp1, "%3.4lf", theta);
		//     fclose(fp1);
		//
		//   }
		//   else if (ph == Phase::RL && side == 1) { //�ܐ您�낵
		//     FILE* fp1 = fopen("Test1.csv", "r");
		//     fscanf(fp1, "%lf", &thetad);
		//     if(thetad == 0){
		//       theta = 0;
		//     }
		//     else{
		//       theta = (p1.x - c1.x -l/2)/r;
		//     }
		//     s = 1 - (r*(theta - sin(theta)) + l/2*(1-cos(theta)))/d;
		//     z = r*(1 - cos(theta)) + l/2*sin(theta);
		//     fclose(fp1);
		//   }
		//   else if(ph == Phase::L && side == 1){
		//     FILE* fp1 = fopen("Test1.csv", "r");
		//     fscanf(fp1, "%lf", &thetad);
		//
		//     if(((c0.x - p1.x) < -l/2) && ((c1.x - p1.x) > l/2)){
		//       if(c < (p1.x - l/2)){
		//         if(thetad==0){
		//           s = 1;
		//           z = 0;
		//         }
		//         else{
		//           theta = (p1.x - c - l/2)/r;
		//           s = 1 - (r*(theta - sin(theta)) + l/2*(1-cos(theta)))/d;
		//           z = r*(1 - cos(theta)) + l/2*sin(theta);
		//         }
		//       }
		//       else if(c > (p1.x + l/2)){
		//         theta = (c - p1.x - l/2)/r;
		//         s = 1 + (r*(theta - sin(theta)) + l/2*(1-cos(theta)))/d;
		//         z = r*(1 - cos(theta)) + l/2*sin(theta);
		//       }
		//       else{
		//         s = 1;
		//         z = 0;
		//       }
		//     }
		//     else{
		//       if(c < (p1.x - l/2)){
		//         if(thetad==0){
		//           s = 1;
		//           z = 0;
		//         }
		//         else{
		//           theta = (p1.x - c - l/2)/r;
		//           s = 1 - (r*(theta - sin(theta)) + l/2*(1-cos(theta)))/d;
		//           z = r*(1 - cos(theta)) + l/2*sin(theta);
		//         }
		//       }
		//       else{
		//         s = 1;
		//         z = 0;
		//       }
		//      }
		//
		//      fclose(fp1);
		//
		//   }



	 //gait_change_smooth
	 // if (param.swingProfile == SwingProfile::Heel_Toe){
		//  real_t _2pi = 2.0*M_PI;
		//  real_t tau = (t - t0) / h;
		//  real_t tau1 = (t - t0) / (td - t0);
		//  real_t tau2 = (t - tdd) / (t1 - tdd);
		//  real_t p00x = p0.x + r0*(theta0 - sin(theta0)) + l/2*(1 - cos(theta0));
		//  real_t p11x = p1.x - (r1*(-theta1 - sin(-theta1)) + l/2*(1 - cos(theta1)));
		//  real_t p00z = r0*(1 - cos(theta0)) + l/2*sin(theta0);
		//  real_t p11z = r1*(1 - cos(theta1)) + l/2*sin(-theta1);
		//  real_t d = p1.x - p0.x;
		//  real_t c = c0.x + (c1.x - c0.x)*tau;
		//  real_t thetadash = 2*pi*tau;
	 //
		//  if((c0.x - p1.x) > l/2){
		// 	 n = 1;
		//  }
		//  else{
		// 	 n = 0;
		//  }
	 //
		//  if(ph == Phase::L && side == 0){
		// 	 FILE* fp = fopen("Test.csv", "r");
		// 	 fscanf(fp, "%d", &thetad);
	 //
		// 	 if(thetad == 0){
		// 		 //real_t alpha = atan(p11z/(p11x-p0.x));
		// 		 real_t a = d/(2*pi);
		// 		 px = p0.x + a*(thetadash-sin(thetadash));
		// 		 z = a*(1-cos(thetadash))/2;
		// 	 }
		// 	 else{
		// 		 real_t a = sqrt((p11x - p00x)*(p11x - p00x) + (p11z - p00z)*(p11z - p00z))/(2*pi);
	 //  		 real_t alpha = atan((p11z-p00z)/(p11x-p00x));
		// 		 px = p00x + a*((thetadash-sin(thetadash))*cos(alpha)-(1-cos(thetadash))*sin(alpha));
	 //  		 z = p00z + a*((1-cos(thetadash))*cos(alpha)/5+(thetadash-sin(thetadash))*sin(alpha));
		// 		 // px = p00x + a*(thetadash-sin(thetadash));
		// 		 // z = p00z + a*(1-cos(thetadash))/3;
		// 	 }
		// 	 s = (px - p0.x)/d;
	 //
		// 	 fclose(fp);
		//  }
		//  else if (ph == Phase::RL && side == 0){
		// 	 if(n == 0){
		// 		 theta = 0;
		// 	 }
		// 	 else{
		// 		 theta_ds = (c0.x - p1.x -l/2)/r0;
		// 		 theta = theta_ds + (theta0 - theta_ds)*tau;
		// 	 }
	 //
		// 	 s = 1 + (r0*(theta - sin(theta)) + l/2*(1-cos(theta)))/d;
		// 	 z = r0*(1 - cos(theta)) + l/2*sin(theta);
	 //
		// 	 FILE* fp = fopen("Test.csv", "w");
		// 	 fprintf(fp, "%d", n);
		// 	 fclose(fp);
	 //
		//  }
		//  else if (ph == Phase::LR && side == 0) { //�ܐ您�낵
		// 	 FILE* fp = fopen("Test.csv", "r");
		// 	 fscanf(fp, "%d", &thetad);
	 //
		// 	 if(thetad == 0){
		// 		 theta = 0;
		// 	 }
		// 	 else{
		// 		 theta_ds = (p1.x - c1.x -l/2)/r1;
		// 		 theta = -theta1 + (theta_ds + theta1)*tau;
		// 	 }
	 //
		// 	 s = 1 - (r1*(theta - sin(theta)) + l/2*(1-cos(theta)))/d;
		// 	 z = r1*(1 - cos(theta)) + l/2*sin(theta);
		// 	 fclose(fp);
		//  }
		//  else if (ph == Phase::R && side == 0){
		// 	 FILE* fp = fopen("Test.csv", "r");
		// 	 fscanf(fp, "%d", &thetad);
	 //
		// 	 if(((c0.x - p1.x) < -l/2) && ((c1.x - p1.x) > l/2)){
		// 		 if(c < (p1.x - l/2)){
		// 			 if(thetad==0){
		// 				 s = 1;
		// 				 z = 0;
		// 			 }
		// 			 else{
		// 				 theta = (p1.x - c - l/2)/r1;
		// 				 s = 1 - (r1*(theta - sin(theta)) + l/2*(1-cos(theta)))/d;
		// 				 z = r1*(1 - cos(theta)) + l/2*sin(theta);
		// 			 }
		// 		 }
		// 		 else if(c > (p1.x + l/2)){
		// 			 theta = (c - p1.x - l/2)/r0;
		// 			 s = 1 + (r0*(theta - sin(theta)) + l/2*(1-cos(theta)))/d;
		// 			 z = r0*(1 - cos(theta)) + l/2*sin(theta);
		// 		 }
		// 		 else{
		// 			 s = 1;
		// 			 z = 0;
		// 		 }
		// 	 }
		// 	 else{
		// 		 if(c < (p1.x - l/2)){
		// 			 if(thetad==0){
		// 				 s = 1;
		// 				 z = 0;
		// 			 }
		// 			 else{
		// 				 theta = (p1.x - c - l/2)/r1;
		// 				 s = 1 - (r1*(theta - sin(theta)) + l/2*(1-cos(theta)))/d;
		// 				 z = r1*(1 - cos(theta)) + l/2*sin(theta);
		// 			 }
		// 		 }
		// 		 else{
		// 			 s = 1;
		// 			 z = 0;
		// 		 }
		// 		}
	 //
		// 		fclose(fp);
	 //
		//  }
	 //
	 //
	 //
	 //
		//  else if (ph == Phase::R && side == 1){
		// 	 FILE* fp1 = fopen("Test1.csv", "r");
		// 	 fscanf(fp1, "%d", &thetad);
	 //
		// 	 if(thetad == 0){
		// 		 //real_t alpha = atan(p11z/(p11x-p0.x));
		// 		 real_t a = d/(2*pi);
		// 		 px = p0.x + a*(thetadash-sin(thetadash));
		// 		 z = a*((1-cos(thetadash)))/2;
		// 	 }
		// 	 else{
		// 		 // real_t a = (p11x - p00x)/(2*pi);
		// 		 // px = p00x + a*(thetadash-sin(thetadash));
		// 		 // z = p00z + a*(1-cos(thetadash))/3;
		// 		 real_t a = sqrt((p11x - p00x)*(p11x - p00x) + (p11z - p00z)*(p11z - p00z))/(2*pi);
	 //  		 real_t alpha = atan((p11z-p00z)/(p11x-p00x));
		// 		 px = p00x + a*((thetadash-sin(thetadash))*cos(alpha)-(1-cos(thetadash))*sin(alpha));
	 //  		 z = p00z + a*((1-cos(thetadash))*cos(alpha)/5+(thetadash-sin(thetadash))*sin(alpha));
		// 	 }
		// 	 s = (px - p0.x)/d;
		// 	 fclose(fp1);
		//  }
		//  else if (ph == Phase::LR && side == 1){
		// 	 if(n == 0){
		// 		 theta = 0;
		// 	 }
		// 	 else{
		// 		 theta_ds = (c0.x - p1.x -l/2)/r0;
		// 		 theta = theta_ds + (theta0 - theta_ds)*tau;
		// 	 }
	 //
		// 	 s = 1 + (r0*(theta - sin(theta)) + l/2*(1-cos(theta)))/d;
		// 	 z = r0*(1 - cos(theta)) + l/2*sin(theta);
	 //
		// 	 FILE* fp1 = fopen("Test1.csv", "w");
		// 	 fprintf(fp1, "%d", n);
		// 	 fclose(fp1);
	 //
		//  }
		//  else if (ph == Phase::RL && side == 1) { //�ܐ您�낵
		// 	 FILE* fp1 = fopen("Test1.csv", "r");
		// 	 fscanf(fp1, "%d", &thetad);
	 //
		// 	 if(thetad == 0){
		// 		 theta = 0;
		// 	 }
		// 	 else{
		// 		 theta_ds = (p1.x - c1.x -l/2)/r1;
		// 		 theta = -theta1 + (theta_ds + theta1)*tau;
		// 	 }
	 //
		// 	 s = 1 - (r1*(theta - sin(theta)) + l/2*(1-cos(theta)))/d;
		// 	 z = r1*(1 - cos(theta)) + l/2*sin(theta);
		// 	 fclose(fp1);
		//  }
		//  else if(ph == Phase::L && side == 1){
		// 	 FILE* fp1 = fopen("Test1.csv", "r");
		// 	 fscanf(fp1, "%d", &thetad);
	 //
		// 	 if(((c0.x - p1.x) < -l/2) && ((c1.x - p1.x) > l/2)){
		// 		 if(c < (p1.x - l/2)){
		// 			 if(thetad==0){
		// 				 s = 1;
		// 				 z = 0;
		// 			 }
		// 			 else{
		// 				 theta = (p1.x - c - l/2)/r1;
		// 				 s = 1 - (r1*(theta - sin(theta)) + l/2*(1-cos(theta)))/d;
		// 				 z = r1*(1 - cos(theta)) + l/2*sin(theta);
		// 			 }
		// 		 }
		// 		 else if(c > (p1.x + l/2)){
		// 			 theta = (c - p1.x - l/2)/r0;
		// 			 s = 1 + (r0*(theta - sin(theta)) + l/2*(1-cos(theta)))/d;
		// 			 z = r0*(1 - cos(theta)) + l/2*sin(theta);
		// 		 }
		// 		 else{
		// 			 s = 1;
		// 			 z = 0;
		// 		 }
		// 	 }
		// 	 else{
		// 		 if(c < (p1.x - l/2)){
		// 			 if(thetad==0){
		// 				 s = 1;
		// 				 z = 0;
		// 			 }
		// 			 else{
		// 				 theta = (p1.x - c - l/2)/r1;
		// 				 s = 1 - (r1*(theta - sin(theta)) + l/2*(1-cos(theta)))/d;
		// 				 z = r1*(1 - cos(theta)) + l/2*sin(theta);
		// 			 }
		// 		 }
		// 		 else{
		// 			 s = 1;
		// 			 z = 0;
		// 		 }
		// 		}
	 //
		// 		fclose(fp1);
	 //
		//  }

		//various_gait
		// if (param.swingProfile == SwingProfile::Heel_Toe){
		//   real_t _2pi = 2.0*M_PI;
		//   real_t tau = (t - t0) / h;
		//   real_t tau1 = (t - t0) / (td - t0);
		//   real_t tau2 = (t - tdd) / (t1 - tdd);
		//   real_t p00x = p0.x + r0*(theta0 - sin(theta0)) + l0*(1 - cos(theta0));
		//   real_t p11x = p1.x - (r1*(-theta1 - sin(-theta1)) + l1*(1 - cos(theta1)));
		//   real_t p00z = r0*(1 - cos(theta0)) + l0*sin(theta0);
		//   real_t p11z = r1*(1 - cos(theta1)) + l1*sin(-theta1);
		//   real_t d = p1.x - p0.x;
		//   real_t c = c0.x + (c1.x - c0.x)*tau;
		//   real_t thetadash = 2*pi*tau;
		//
		//   if((c0.x - p1.x) > l0){
		//  	 n = 1;
		//   }
		//   else{
		//  	 n = 0;
		//   }
		//
		//   if(ph == Phase::L && side == 0){
		//  	 FILE* fp = fopen("Test.csv", "r");
		//  	 fscanf(fp, "%d", &thetad);
		//
		//  	 if(thetad == 0){
		//  		 //real_t alpha = atan(p11z/(p11x-p0.x));
		//  		 real_t a = d/(2*pi);
		//  		 px = p0.x + a*(thetadash-sin(thetadash));
		//  		 z = a*(1-cos(thetadash))/2;
		//  	 }
		//  	 else{
		//  		 real_t a = sqrt((p11x - p00x)*(p11x - p00x) + (p11z - p00z)*(p11z - p00z))/(2*pi);
		//  		 real_t alpha = atan((p11z-p00z)/(p11x-p00x));
		//  		 px = p00x + a*((thetadash-sin(thetadash))*cos(alpha)-(1-cos(thetadash))*sin(alpha));
		//  		 z = p00z + a*((1-cos(thetadash))*cos(alpha)/5+(thetadash-sin(thetadash))*sin(alpha));
		//  		 // px = p00x + a*(thetadash-sin(thetadash));
		//  		 // z = p00z + a*(1-cos(thetadash))/3;
		//  	 }
		//  	 s = (px - p0.x)/d;
		//
		//  	 fclose(fp);
		//   }
		//   else if (ph == Phase::RL && side == 0){
		//  	 if(n == 0){
		//  		 theta = 0;
		//  	 }
		//  	 else{
		//  		 theta_ds = (c0.x - p1.x -l0)/r0;
		//  		 theta = theta_ds + (theta0 - theta_ds)*tau;
		//  	 }
		//
		//  	 s = 1 + (r0*(theta - sin(theta)) + l0*(1-cos(theta)))/d;
		//  	 z = r0*(1 - cos(theta)) + l0*sin(theta);
		//
		//  	 FILE* fp = fopen("Test.csv", "w");
		//  	 fprintf(fp, "%d", n);
		//  	 fclose(fp);
		//
		//   }
		//   else if (ph == Phase::LR && side == 0) { //�ܐ您�낵
		//  	 FILE* fp = fopen("Test.csv", "r");
		//  	 fscanf(fp, "%d", &thetad);
		//
		//  	 if(thetad == 0){
		//  		 theta = 0;
		//  	 }
		//  	 else{
		//  		 theta_ds = (p1.x - c1.x -l1)/r1;
		//  		 theta = -theta1 + (theta_ds + theta1)*tau;
		//  	 }
		//
		//  	 s = 1 - (r1*(theta - sin(theta)) + l1*(1-cos(theta)))/d;
		//  	 z = r1*(1 - cos(theta)) + l1*sin(theta);
		//  	 fclose(fp);
		//   }
		//   else if (ph == Phase::R && side == 0){
		//  	 FILE* fp = fopen("Test.csv", "r");
		//  	 fscanf(fp, "%d", &thetad);
		//
		//  	 if(((c0.x - p1.x) < -l1) && ((c1.x - p1.x) > l0)){
		//  		 if(c < (p1.x - l1)){
		//  			 if(thetad==0){
		//  				 s = 1;
		//  				 z = 0;
		//  			 }
		//  			 else{
		//  				 theta = (p1.x - c - l1)/r1;
		//  				 s = 1 - (r1*(theta - sin(theta)) + l1*(1-cos(theta)))/d;
		//  				 z = r1*(1 - cos(theta)) + l1*sin(theta);
		//  			 }
		//  		 }
		//  		 else if(c > (p1.x + l0)){
		//  			 theta = (c - p1.x - l0)/r0;
		//  			 s = 1 + (r0*(theta - sin(theta)) + l0*(1-cos(theta)))/d;
		//  			 z = r0*(1 - cos(theta)) + l0*sin(theta);
		//  		 }
		//  		 else{
		//  			 s = 1;
		//  			 z = 0;
		//  		 }
		//  	 }
		//  	 else{
		//  		 if(c < (p1.x - l1)){
		//  			 if(thetad==0){
		//  				 s = 1;
		//  				 z = 0;
		//  			 }
		//  			 else{
		//  				 theta = (p1.x - c - l1)/r1;
		//  				 s = 1 - (r1*(theta - sin(theta)) + l1*(1-cos(theta)))/d;
		//  				 z = r1*(1 - cos(theta)) + l1*sin(theta);
		//  			 }
		//  		 }
		//  		 else{
		//  			 s = 1;
		//  			 z = 0;
		//  		 }
		//  		}
		//
		//  		fclose(fp);
		//
		//   }
		//
		//
		//
		//
		//   else if (ph == Phase::R && side == 1){
		//  	 FILE* fp1 = fopen("Test1.csv", "r");
		//  	 fscanf(fp1, "%d", &thetad);
		//
		//  	 if(thetad == 0){
		//  		 //real_t alpha = atan(p11z/(p11x-p0.x));
		//  		 real_t a = d/(2*pi);
		//  		 px = p0.x + a*(thetadash-sin(thetadash));
		//  		 z = a*((1-cos(thetadash)))/2;
		//  	 }
		//  	 else{
		//  		 // real_t a = (p11x - p00x)/(2*pi);
		//  		 // px = p00x + a*(thetadash-sin(thetadash));
		//  		 // z = p00z + a*(1-cos(thetadash))/3;
		//  		 real_t a = sqrt((p11x - p00x)*(p11x - p00x) + (p11z - p00z)*(p11z - p00z))/(2*pi);
		//  		 real_t alpha = atan((p11z-p00z)/(p11x-p00x));
		//  		 px = p00x + a*((thetadash-sin(thetadash))*cos(alpha)-(1-cos(thetadash))*sin(alpha));
		//  		 z = p00z + a*((1-cos(thetadash))*cos(alpha)/5+(thetadash-sin(thetadash))*sin(alpha));
		//  	 }
		//  	 s = (px - p0.x)/d;
		//  	 fclose(fp1);
		//   }
		//   else if (ph == Phase::LR && side == 1){
		//  	 if(n == 0){
		//  		 theta = 0;
		//  	 }
		//  	 else{
		//  		 theta_ds = (c0.x - p1.x -l0)/r0;
		//  		 theta = theta_ds + (theta0 - theta_ds)*tau;
		//  	 }
		//
		//  	 s = 1 + (r0*(theta - sin(theta)) + l0*(1-cos(theta)))/d;
		//  	 z = r0*(1 - cos(theta)) + l0*sin(theta);
		//
		//  	 FILE* fp1 = fopen("Test1.csv", "w");
		//  	 fprintf(fp1, "%d", n);
		//  	 fclose(fp1);
		//
		//   }
		//   else if (ph == Phase::RL && side == 1) { //�ܐ您�낵
		//  	 FILE* fp1 = fopen("Test1.csv", "r");
		//  	 fscanf(fp1, "%d", &thetad);
		//
		//  	 if(thetad == 0){
		//  		 theta = 0;
		//  	 }
		//  	 else{
		//  		 theta_ds = (p1.x - c1.x -l1)/r1;
		//  		 theta = -theta1 + (theta_ds + theta1)*tau;
		//  	 }
		//
		//  	 s = 1 - (r1*(theta - sin(theta)) + l1*(1-cos(theta)))/d;
		//  	 z = r1*(1 - cos(theta)) + l1*sin(theta);
		//  	 fclose(fp1);
		//   }
		//   else if(ph == Phase::L && side == 1){
		//  	 FILE* fp1 = fopen("Test1.csv", "r");
		//  	 fscanf(fp1, "%d", &thetad);
		//
		//  	 if(((c0.x - p1.x) < -l1) && ((c1.x - p1.x) > l0)){
		//  		 if(c < (p1.x - l1)){
		//  			 if(thetad==0){
		//  				 s = 1;
		//  				 z = 0;
		//  			 }
		//  			 else{
		//  				 theta = (p1.x - c - l1)/r1;
		//  				 s = 1 - (r1*(theta - sin(theta)) + l1*(1-cos(theta)))/d;
		//  				 z = r1*(1 - cos(theta)) + l1*sin(theta);
		//  			 }
		//  		 }
		//  		 else if(c > (p1.x + l0)){
		//  			 theta = (c - p1.x - l0)/r0;
		//  			 s = 1 + (r0*(theta - sin(theta)) + l0*(1-cos(theta)))/d;
		//  			 z = r0*(1 - cos(theta)) + l0*sin(theta);
		//  		 }
		//  		 else{
		//  			 s = 1;
		//  			 z = 0;
		//  		 }
		//  	 }
		//  	 else{
		//  		 if(c < (p1.x - l1)){
		//  			 if(thetad==0){
		//  				 s = 1;
		//  				 z = 0;
		//  			 }
		//  			 else{
		//  				 theta = (p1.x - c - l1)/r1;
		//  				 s = 1 - (r1*(theta - sin(theta)) + l1*(1-cos(theta)))/d;
		//  				 z = r1*(1 - cos(theta)) + l1*sin(theta);
		//  			 }
		//  		 }
		//  		 else{
		//  			 s = 1;
		//  			 z = 0;
		//  		 }
		//  		}
		//
		//  		fclose(fp1);
		//
		//   }


	  //foot_shape_change
		if (param.swingProfile == SwingProfile::Heel_Toe){
		 real_t _2pi = 2.0*M_PI;
		 real_t tau = (t - t0) / h;
		 real_t tau1 = (t - t0) / (td - t0);
		 real_t tau2 = (t - tdd) / (t1 - tdd);
		 real_t p00x = (p0.x + dev) + r0*(theta0 - sin(theta0)) + l0*(1 - cos(theta0));
		 real_t p11x = (p1.x + dev) - (r1*(-theta1 - sin(-theta1)) + l1*(1 - cos(theta1)));
		 real_t p00z = r0*(1 - cos(theta0)) + l0*sin(theta0);
		 real_t p11z = r1*(1 - cos(theta1)) + l1*sin(-theta1);
		 real_t d = p1.x - p0.x;
		 real_t c = c0.x + (c1.x - c0.x)*tau;
		 real_t thetadash = 2*pi*tau;

		 if((c0.x - (p1.x + dev)) > l0){
			n = 1;
		 }
		 else{
			n = 0;
		 }

		 if(ph == Phase::L && side == 0){
			FILE* fp = fopen("Test.csv", "r");
			fscanf(fp, "%d", &thetad);

			if(thetad == 0){
				//real_t alpha = atan(p11z/(p11x-p0.x));
				real_t a = d/(2*pi);
				px = (p0.x + dev) + a*(thetadash-sin(thetadash));
				z = a*(1-cos(thetadash))/2;
			}
			else{
				real_t a = sqrt((p11x - p00x)*(p11x - p00x) + (p11z - p00z)*(p11z - p00z))/(2*pi);
				real_t alpha = atan((p11z-p00z)/(p11x-p00x));
				px = p00x + a*((thetadash-sin(thetadash))*cos(alpha)-(1-cos(thetadash))*sin(alpha));
				z = p00z + a*((1-cos(thetadash))*cos(alpha)/5+(thetadash-sin(thetadash))*sin(alpha));
				// px = p00x + a*(thetadash-sin(thetadash));
				// z = p00z + a*(1-cos(thetadash))/3;
			}
			s = (px - (p0.x + dev))/d;

			fclose(fp);
		 }
		 else if (ph == Phase::RL && side == 0){
			if(n == 0){
				theta = 0;
			}
			else{
				theta_ds = (c0.x - (p1.x + dev) -l0)/r0;
				theta = theta_ds + (theta0 - theta_ds)*tau;
			}

			s = 1 + (r0*(theta - sin(theta)) + l0*(1-cos(theta)))/d;
			z = r0*(1 - cos(theta)) + l0*sin(theta);

			FILE* fp = fopen("Test.csv", "w");
			fprintf(fp, "%d", n);
			fclose(fp);

		 }
		 else if (ph == Phase::LR && side == 0) { //�ܐ您�낵
			FILE* fp = fopen("Test.csv", "r");
			fscanf(fp, "%d", &thetad);

			if(thetad == 0){
				theta = 0;
			}
			else{
				theta_ds = ((p1.x + dev) - c1.x -l1)/r1;
				theta = -theta1 + (theta_ds + theta1)*tau;
			}

			s = 1 - (r1*(theta - sin(theta)) + l1*(1-cos(theta)))/d;
			z = r1*(1 - cos(theta)) + l1*sin(theta);
			fclose(fp);
		 }
		 else if (ph == Phase::R && side == 0){
			FILE* fp = fopen("Test.csv", "r");
			fscanf(fp, "%d", &thetad);

			if(((c0.x - (p1.x + dev)) < -l1) && ((c1.x - (p1.x + dev)) > l0)){
				if(c < ((p1.x + dev) - l1)){
					if(thetad==0){
						s = 1;
						z = 0;
					}
					else{
						theta = ((p1.x + dev) - c - l1)/r1;
						s = 1 - (r1*(theta - sin(theta)) + l1*(1-cos(theta)))/d;
						z = r1*(1 - cos(theta)) + l1*sin(theta);
					}
				}
				else if(c > ((p1.x + dev) + l0)){
					theta = (c - (p1.x + dev) - l0)/r0;
					s = 1 + (r0*(theta - sin(theta)) + l0*(1-cos(theta)))/d;
					z = r0*(1 - cos(theta)) + l0*sin(theta);
				}
				else{
					s = 1;
					z = 0;
				}
			}
			else{
				if(c < ((p1.x + dev) - l1)){
					if(thetad==0){
						s = 1;
						z = 0;
					}
					else{
						theta = ((p1.x + dev) - c - l1)/r1;
						s = 1 - (r1*(theta - sin(theta)) + l1*(1-cos(theta)))/d;
						z = r1*(1 - cos(theta)) + l1*sin(theta);
					}
				}
				else{
					s = 1;
					z = 0;
				}
			 }

			 fclose(fp);

		 }




		 else if (ph == Phase::R && side == 1){
			FILE* fp1 = fopen("Test1.csv", "r");
			fscanf(fp1, "%d", &thetad);

			if(thetad == 0){
				//real_t alpha = atan(p11z/(p11x-p0.x));
				real_t a = d/(2*pi);
				px = (p0.x + dev) + a*(thetadash-sin(thetadash));
				z = a*((1-cos(thetadash)))/2;
			}
			else{
				// real_t a = (p11x - p00x)/(2*pi);
				// px = p00x + a*(thetadash-sin(thetadash));
				// z = p00z + a*(1-cos(thetadash))/3;
				real_t a = sqrt((p11x - p00x)*(p11x - p00x) + (p11z - p00z)*(p11z - p00z))/(2*pi);
				real_t alpha = atan((p11z-p00z)/(p11x-p00x));
				px = p00x + a*((thetadash-sin(thetadash))*cos(alpha)-(1-cos(thetadash))*sin(alpha));
				z = p00z + a*((1-cos(thetadash))*cos(alpha)/5+(thetadash-sin(thetadash))*sin(alpha));
			}
			s = (px - (p0.x + dev))/d;
			fclose(fp1);
		 }
		 else if (ph == Phase::LR && side == 1){
			if(n == 0){
				theta = 0;
			}
			else{
				theta_ds = (c0.x - (p1.x + dev) -l0)/r0;
				theta = theta_ds + (theta0 - theta_ds)*tau;
			}

			s = 1 + (r0*(theta - sin(theta)) + l0*(1-cos(theta)))/d;
			z = r0*(1 - cos(theta)) + l0*sin(theta);

			FILE* fp1 = fopen("Test1.csv", "w");
			fprintf(fp1, "%d", n);
			fclose(fp1);

		 }
		 else if (ph == Phase::RL && side == 1) { //�ܐ您�낵
			FILE* fp1 = fopen("Test1.csv", "r");
			fscanf(fp1, "%d", &thetad);

			if(thetad == 0){
				theta = 0;
			}
			else{
				theta_ds = ((p1.x + dev) - c1.x -l1)/r1;
				theta = -theta1 + (theta_ds + theta1)*tau;
			}

			s = 1 - (r1*(theta - sin(theta)) + l1*(1-cos(theta)))/d;
			z = r1*(1 - cos(theta)) + l1*sin(theta);
			fclose(fp1);
		 }
		 else if(ph == Phase::L && side == 1){
			FILE* fp1 = fopen("Test1.csv", "r");
			fscanf(fp1, "%d", &thetad);

			if(((c0.x - (p1.x + dev)) < -l1) && ((c1.x - (p1.x + dev)) > l0)){
				if(c < ((p1.x + dev) - l1)){
					if(thetad==0){
						s = 1;
						z = 0;
					}
					else{
						theta = ((p1.x + dev) - c - l1)/r1;
						s = 1 - (r1*(theta - sin(theta)) + l1*(1-cos(theta)))/d;
						z = r1*(1 - cos(theta)) + l1*sin(theta);
					}
				}
				else if(c > ((p1.x + dev) + l0)){
					theta = (c - (p1.x + dev) - l0)/r0;
					s = 1 + (r0*(theta - sin(theta)) + l0*(1-cos(theta)))/d;
					z = r0*(1 - cos(theta)) + l0*sin(theta);
				}
				else{
					s = 1;
					z = 0;
				}
			}
			else{
				if(c < ((p1.x + dev) - l1)){
					if(thetad==0){
						s = 1;
						z = 0;
					}
					else{
						theta = ((p1.x + dev) - c - l1)/r1;
						s = 1 - (r1*(theta - sin(theta)) + l1*(1-cos(theta)))/d;
						z = r1*(1 - cos(theta)) + l1*sin(theta);
					}
				}
				else{
					s = 1;
					z = 0;
				}
			 }

			 fclose(fp1);

		 }

    //foot_shape_diffferent
		// if (param.swingProfile == SwingProfile::Heel_Toe){
		//   real_t _2pi = 2.0*M_PI;
		//   real_t tau = (t - t0) / h;
		//   real_t tau1 = (t - t0) / (td - t0);
		//   real_t tau2 = (t - tdd) / (t1 - tdd);
		//   real_t p00x = p0.x + r0*(theta0 - sin(theta0)) + l0*(1 - cos(theta0));
		//   real_t p11x = p1.x - (r1*(-theta1 - sin(-theta1)) + l1*(1 - cos(theta1)));
		//   real_t p00z = r0*(1 - cos(theta0)) + l0*sin(theta0);
		//   real_t p11z = r1*(1 - cos(theta1)) + l1*sin(-theta1);
		//   real_t d = p1.x - p0.x;
		//   real_t c = c0.x + (c1.x - c0.x)*tau;
		//   real_t thetadash = 2*pi*tau;
		//
		//   if((c0.x - p1.x) > l0 + r0*theta0){
		//  	 n = 1;
		//   }
		//   else{
		//  	 n = 0;
		//   }
		//
		//   if(ph == Phase::L && side == 0){
		//  	 FILE* fp = fopen("Test.csv", "r");
		//  	 fscanf(fp, "%d", &thetad);
		//
		//  	 if(thetad == 0){
		//  		 //real_t alpha = atan(p11z/(p11x-p0.x));
		//  		 real_t a = d/(2*pi);
		//  		 px = p0.x + a*(thetadash-sin(thetadash));
		//  		 z = a*(1-cos(thetadash))/2;
		//  	 }
		//  	 else{
		//  		 real_t a = sqrt((p11x - p00x)*(p11x - p00x) + (p11z - p00z)*(p11z - p00z))/(2*pi);
		//  		 real_t alpha = atan((p11z-p00z)/(p11x-p00x));
		//  		 px = p00x + a*((thetadash-sin(thetadash))*cos(alpha)-(1-cos(thetadash))*sin(alpha));
		//  		 z = p00z + a*((1-cos(thetadash))*cos(alpha)/5+(thetadash-sin(thetadash))*sin(alpha));
		//  		 // px = p00x + a*(thetadash-sin(thetadash));
		//  		 // z = p00z + a*(1-cos(thetadash))/3;
		//  	 }
		//  	 s = (px - p0.x)/d;
		//
		//  	 fclose(fp);
		//   }
		//   else if (ph == Phase::RL && side == 0){
		//  	 if(n == 0){
		//  		 theta = 0;
		//  	 }
		//  	 else{
		//  		 theta = theta0;
		//  	 }
		//
		//  	 s = 1 + (r0*(theta - sin(theta)) + l0*(1-cos(theta)))/d;
		//  	 z = r0*(1 - cos(theta)) + l0*sin(theta);
		//
		//  	 FILE* fp = fopen("Test.csv", "w");
		//  	 fprintf(fp, "%d", n);
		//  	 fclose(fp);
		//
		//   }
		//   else if (ph == Phase::LR && side == 0) { //�ܐ您�낵
		//  	 FILE* fp = fopen("Test.csv", "r");
		//  	 fscanf(fp, "%d", &thetad);
		//
		//  	 if(thetad == 0){
		//  		 theta = 0;
		//  	 }
		//  	 else{
		// 		 theta_ds = (p1.x - c1.x -l1)/r1;
		//  		 theta = -theta1 + (theta_ds + theta1)*tau;
		//  	 }
		//
		//  	 s = 1 - (r1*(theta - sin(theta)) + l1*(1-cos(theta)))/d;
		//  	 z = r1*(1 - cos(theta)) + l1*sin(theta);
		//  	 fclose(fp);
		//   }
		//   else if (ph == Phase::R && side == 0){
		//  	 FILE* fp = fopen("Test.csv", "r");
		//  	 fscanf(fp, "%d", &thetad);
		//
		//  	 if(((c0.x - p1.x) < -l1) && ((c1.x - p1.x) > l0 + r0*theta0)){
		//  		 if(c < (p1.x - l1)){
		//  			 if(thetad==0){
		//  				 s = 1;
		//  				 z = 0;
		//  			 }
		//  			 else{
		//  				 theta = (p1.x - c - l1)/r1;
		//  				 s = 1 - (r1*(theta - sin(theta)) + l1*(1-cos(theta)))/d;
		//  				 z = r1*(1 - cos(theta)) + l1*sin(theta);
		//  			 }
		//  		 }
		//  		 else if(c > (p1.x + l0) && c < (p1.x + l0 + r0*theta0)){
		//  			 theta = (c - p1.x - l0)/r0;
		//  			 s = 1 + (r0*(theta - sin(theta)) + l0*(1-cos(theta)))/d;
		//  			 z = r0*(1 - cos(theta)) + l0*sin(theta);
		//  		 }
		// 		 else if(c > (p1.x + l0 + r0*theta0)){
		// 			 theta = theta0;
		// 			 s = 1 + (r0*(theta - sin(theta)) + l0*(1-cos(theta)))/d;
		//  			 z = r0*(1 - cos(theta)) + l0*sin(theta);
		// 		 }
		//  		 else{
		//  			 s = 1;
		//  			 z = 0;
		//  		 }
		//  	 }
		//  	 else{
		//  		 if(c < (p1.x - l1)){
		//  			 if(thetad==0){
		//  				 s = 1;
		//  				 z = 0;
		//  			 }
		//  			 else{
		//  				 theta = (p1.x - c - l1)/r1;
		//  				 s = 1 - (r1*(theta - sin(theta)) + l1*(1-cos(theta)))/d;
		//  				 z = r1*(1 - cos(theta)) + l1*sin(theta);
		//  			 }
		//  		 }
		//  		 else{
		//  			 s = 1;
		//  			 z = 0;
		//  		 }
		//  		}
		//
		//  		fclose(fp);
		//
		//   }
		//
		//
		//
		//
		//   else if (ph == Phase::R && side == 1){
		//  	 FILE* fp1 = fopen("Test1.csv", "r");
		//  	 fscanf(fp1, "%d", &thetad);
		//
		//  	 if(thetad == 0){
		//  		 //real_t alpha = atan(p11z/(p11x-p0.x));
		//  		 real_t a = d/(2*pi);
		//  		 px = p0.x + a*(thetadash-sin(thetadash));
		//  		 z = a*((1-cos(thetadash)))/2;
		//  	 }
		//  	 else{
		//  		 // real_t a = (p11x - p00x)/(2*pi);
		//  		 // px = p00x + a*(thetadash-sin(thetadash));
		//  		 // z = p00z + a*(1-cos(thetadash))/3;
		//  		 real_t a = sqrt((p11x - p00x)*(p11x - p00x) + (p11z - p00z)*(p11z - p00z))/(2*pi);
		//  		 real_t alpha = atan((p11z-p00z)/(p11x-p00x));
		//  		 px = p00x + a*((thetadash-sin(thetadash))*cos(alpha)-(1-cos(thetadash))*sin(alpha));
		//  		 z = p00z + a*((1-cos(thetadash))*cos(alpha)/5+(thetadash-sin(thetadash))*sin(alpha));
		//  	 }
		//  	 s = (px - p0.x)/d;
		//  	 fclose(fp1);
		//   }
		//   else if (ph == Phase::LR && side == 1){
		//  	 if(n == 0){
		//  		 theta = 0;
		//  	 }
		//  	 else{
		//  		 theta = theta0;
		//  	 }
		//
		//  	 s = 1 + (r0*(theta - sin(theta)) + l0*(1-cos(theta)))/d;
		//  	 z = r0*(1 - cos(theta)) + l0*sin(theta);
		//
		//  	 FILE* fp1 = fopen("Test1.csv", "w");
		//  	 fprintf(fp1, "%d", n);
		//  	 fclose(fp1);
		//
		//   }
		//   else if (ph == Phase::RL && side == 1) { //�ܐ您�낵
		//  	 FILE* fp1 = fopen("Test1.csv", "r");
		//  	 fscanf(fp1, "%d", &thetad);
		//
		//  	 if(thetad == 0){
		//  		 theta = 0;
		//  	 }
		//  	 else{
		//  		 theta_ds = (p1.x - c1.x -l1)/r1;
		//  		 theta = -theta1 + (theta_ds + theta1)*tau;
		//  	 }
		//
		//  	 s = 1 - (r1*(theta - sin(theta)) + l1*(1-cos(theta)))/d;
		//  	 z = r1*(1 - cos(theta)) + l1*sin(theta);
		//  	 fclose(fp1);
		//   }
		//   else if(ph == Phase::L && side == 1){
		//  	 FILE* fp1 = fopen("Test1.csv", "r");
		//  	 fscanf(fp1, "%d", &thetad);
		//
		//  	 if(((c0.x - p1.x) < -l1) && ((c1.x - p1.x) > l0 + r0*theta0)){
		//  		 if(c < (p1.x - l1)){
		//  			 if(thetad==0){
		//  				 s = 1;
		//  				 z = 0;
		//  			 }
		//  			 else{
		//  				 theta = (p1.x - c - l1)/r1;
		//  				 s = 1 - (r1*(theta - sin(theta)) + l1*(1-cos(theta)))/d;
		//  				 z = r1*(1 - cos(theta)) + l1*sin(theta);
		//  			 }
		//  		 }
		//  		 else if(c > (p1.x + l0) && c < (p1.x + l0 + r0*theta0)){
		//  			 theta = (c - p1.x - l0)/r0;
		//  			 s = 1 + (r0*(theta - sin(theta)) + l0*(1-cos(theta)))/d;
		//  			 z = r0*(1 - cos(theta)) + l0*sin(theta);
		//  		 }
		// 		 else if(c > (p1.x + l0 + r0*theta0)){
		// 			 theta = theta0;
		// 			 s = 1 + (r0*(theta - sin(theta)) + l0*(1-cos(theta)))/d;
		//  			 z = r0*(1 - cos(theta)) + l0*sin(theta);
		// 		 }
		//  		 else{
		//  			 s = 1;
		//  			 z = 0;
		//  		 }
		//  	 }
		//  	 else{
		//  		 if(c < (p1.x - l1)){
		//  			 if(thetad==0){
		//  				 s = 1;
		//  				 z = 0;
		//  			 }
		//  			 else{
		//  				 theta = (p1.x - c - l1)/r1;
		//  				 s = 1 - (r1*(theta - sin(theta)) + l1*(1-cos(theta)))/d;
		//  				 z = r1*(1 - cos(theta)) + l1*sin(theta);
		//  			 }
		//  		 }
		//  		 else{
		//  			 s = 1;
		//  			 z = 0;
		//  		 }
		//  		}
		//
		//  		fclose(fp1);
		//
		//   }





		}
		pt.x = (1 - s)*(p0.x + dev) + s * (p1.x + dev);
		pt.y = p0.y + tau*(p1.y-p0.y);
	}
	else {
		pt = key0->var_foot_pos_t[side]->val;
	}

		return vec3_t(pt.x, pt.y, z);
	}

	//�����p�x
	quat_t BipedLIP::FootOri(real_t t, int side) {
		BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
		BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

		real_t ot;
		quat_t qt;

		if (key1) {
			real_t dt = t - key0->var_time->val;
			real_t tau = key0->var_duration->val;
			real_t o0 = key0->var_foot_pos_r[side]->val;
			real_t o1 = key1->var_foot_pos_r[side]->val;

			ot = o0 + (o1 - o0)*(dt / tau);
		}
		else {
			ot = key0->var_foot_pos_r[side]->val;
		}

		qt = quat_t::Rot(ot, 'z');

		return qt;
	}



	real_t BipedLIP::AnklePitch(real_t t, int side) {
	  BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
	  BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;
	  real_t       t0 = key0->tick->time;
	  real_t       t1 = key1->tick->time;
	  real_t       h = t1 - t0;
	  real_t		   hspan = 0.3*h;
	  real_t		   t00 = t0 + hspan;
	  real_t       t11 = t1 - hspan;
		real_t       td = (t0 + t1)/10;
		real_t       tdd = (t0 + t1)*29/30;

	  real_t theta = 0.0;
	  real_t theta0 = 30*pi/180;
		real_t theta1 = -30*pi/180;
		//real_t thetaop = 10*pi/180;
		real_t thetatoe = 10*pi/180;
		real_t thetaheel = 10*pi/180;
		real_t theta_ds;
		int thetad;

		//real_t l = 0.16;
		// //real_t r = 0.02/thetaop;
		// real_t r0 = 0.01/thetatoe;
		// real_t r1 = 0.01/thetaheel;

		real_t l0 = 0.08;
		real_t l1 = 0.08;
		real_t L = 0.20;
		//real_t r = 0.025;
		//real_t r = 0.02/thetaop;
		//real_t r0 = (L/2-l0)/thetatoe;
		//real_t r1 = (L/2-l1)/thetaheel;
		real_t r0 = 0.03;
		real_t r1 = 0.03;

		real_t dev = 0.0;

		// real_t thetaps = 10*pi/180;
		// real_t l2 = 0.08;
		// real_t r = (L/2-l0)/thetaps;

		// real_t l0 = 0.02;
		// real_t l1 = 0.08;
		// real_t l2 = 0.02;
		// real_t L = 0.20;
		// //real_t r = 0.025;
		// //real_t r = 0.02/thetaop;
		// real_t r0 = (L/2-l0-l2)/theta0;
		// real_t r1 = (L/2-l1)/thetaheel;




	  if (key0->prev) {
	    real_t dt = t - key0->var_time->val;
	    real_t tau = key0->var_duration->val;
			vec2_t p0 = key0->var_foot_pos_t[side]->val;
			vec2_t p1 = key1->var_foot_pos_t[side]->val;
			vec2_t c0 = key0->var_cop_pos->val;
			vec2_t c1 = key1->var_cop_pos->val;
			int ph = phase[key0->tick->idx];

			// if((ph == Phase::R && side == 1) || (ph == Phase::L && side == 0)){
			// 	if((c0.x - p1.x < -l/2) && (c1.x - p1.x > l/2)){
			// 		param.swingProfile = SwingProfile::Heel_Toe;
			// 	}
			// 	else{
			// 		param.swingProfile = SwingProfile::Cycloid;
			// 	}
			// }



	    // �P�r�x�����̗V�r
			//kataoka saigen
	  //   int ph = phase[key0->tick->idx];
	  //   if (param.swingProfile == SwingProfile::Heel_Toe){
	  //     real_t _2pi = 2.0*M_PI;
	  //     real_t tau = (t - t0) / h;
	  //     real_t tau0 = (t - t0) / (t00 - t0);
	  //     real_t tau1 = (t - t11) / (t1 - t11);
	  //     real_t tau2 = (t - t00) / (t11 - t00);
		//
		//
	  //     if ((ph == Phase::R && side == 1) || (ph == Phase::L && side == 0)){
	  //       theta = theta0 + tau*(theta1 - theta0);
	  //     }
	  //     else if ((ph == Phase::RL && side == 0) || (ph == Phase::LR && side == 1)){
	  //       if(t<= t11){
	  //         theta=0;
	  //       }
	  //       else{
	  //         theta = tau1 * theta0;
	  //       }
	  //     }
	  //     else if ((ph == Phase::RL && side == 1) || (ph == Phase::LR && side == 0)) { //�ܐ您�낵
	  //       if (t <= t00) {
	  //         theta = (1 - tau0)*theta1;
	  //       }
	  //       else {
	  //         theta=0;
	  //       }
	  //     }
	  //     else{
	  //       theta=0;
	  //     }
	  //   }
	  // }


			// if (key1) {
			// 	real_t dt = t - key0->var_time->val;
			// 	real_t tau = key0->var_duration->val;
			// 	vec2_t p0 = key0->var_foot_pos_t[side]->val;
			// 	vec2_t p1 = key1->var_foot_pos_t[side]->val;
			//
			// 	// �P�r�x�����̗V�r
			// 	int ph = phase[key0->tick->idx];
			// 	if ((ph == Phase::R && side == 1) || (ph == Phase::L && side == 0)) { //(�E���Б��x���� or �����Б��x����)
			// 		  real_t _2pi = 2.0*M_PI;
			// 			real_t tau = (t - t0) / h;
			// 			real_t tau0 = (t - t0) / (t00 - t0);
			// 			real_t tau1 = (t - t11) / (t1 - t11);
			// 			real_t tau2 = (t - t00) / (t11 - t00);
			//
			//
			// 			if ((t >= t0) && (t <= t00)){
			// 				theta = tau0 * theta0;
			// 			}
			// 			else if ((t >= t11) && (t <= t1)){
			// 				theta = (1 - tau1)*theta1;
			// 			}
			// 			else {
			// 				theta = theta0 + tau2*(theta1 - theta0);
			// 			}
			// 		}
			// 	}


			//new
			//int ph = phase[key0->tick->idx];
			// if (param.swingProfile == SwingProfile::Heel_Toe){
			// 	real_t _2pi = 2.0*M_PI;
			// 	real_t tau = (t - t0) / h;
			// 	real_t tau1 = (t - t0) / (td - t0);
			// 	real_t tau2 = (t - tdd) / (t1 - tdd);
			//
			//
			// 	if ((ph == Phase::R && side == 1) || (ph == Phase::L && side == 0)){
			// 		theta = theta0 + tau*(theta1 - theta0);
			// 	}
			// 	else if ((ph == Phase::RL && side == 0) || (ph == Phase::LR && side == 1)){
			// 		theta = thetaps + tau*(theta0-thetaps);
			// 	}
			// 	else if ((ph == Phase::RL && side == 1) || (ph == Phase::LR && side == 0)) { //�ܐ您�낵
			// 		if (t <= td) {
			// 			theta = (1-tau1)*theta1;
			// 		}
			// 		else {
			// 			theta = 0;
			// 		}
			// 	}
			// 	else{
			// 		if (t <= tdd) {
			// 			theta = 0;
			// 		}
			// 		else {
			// 			theta = tau2 * thetaps;
			// 		}
			// 	}





		// //gait_change
		// if (param.swingProfile == SwingProfile::Heel_Toe){
		// 	real_t _2pi = 2.0*M_PI;
		// 	real_t tau = (t - t0) / h;
		// 	// real_t tau1 = (t - t0) / (td - t0);
		// 	// real_t tau2 = (t - tdd) / (t1 - tdd);
	  //   real_t c = c0.x + (c1.x - c0.x)*tau;
		//
		//
		//
		//
		// 	if(ph == Phase::L && side == 0){
		//     FILE* fp = fopen("Test.csv", "r");
		//     fscanf(fp, "%lf", &thetad);
		//
		//     if(thetad == 0){
		//       theta = 0;
		//     }
		//     else{
		//       theta = theta0 + tau*(theta1 - theta0);
		//     }
		//
		//     fclose(fp);
		//   }
		//   else if (ph == Phase::RL && side == 0){
		// 		FILE* fp = fopen("Test.csv", "r");
		//     fscanf(fp, "%lf", &thetad);
		// 		theta = thetad;
		//     fclose(fp);
		//   }
		//   else if (ph == Phase::LR && side == 0) { //�ܐ您�낵
		//     FILE* fp = fopen("Test.csv", "r");
		//     fscanf(fp, "%lf", &thetad);
		//
		//     if(thetad == 0){
		//       theta = 0;
		//     }
		//     else{
		//       theta = -(p1.x - c1.x -l/2)/r;
		//     }
		//
		//     fclose(fp);
		//   }
		//   else if (ph == Phase::R && side == 0){
		//     FILE* fp = fopen("Test.csv", "r");
		//     fscanf(fp, "%lf", &thetad);
		//
		//     if(((c0.x - p1.x) < -l/2) && ((c1.x - p1.x) > l/2)){
		//       if(c < (p1.x - l/2)){
		//         if(thetad==0){
		//           theta = 0;
		//         }
		//         else{
		//           theta = -(p1.x - c - l/2)/r;
		//         }
		//       }
		//       else if(c > (p1.x + l/2)){
		//         theta = (c - p1.x - l/2)/r;
		//       }
		//       else{
		//         theta = 0;
		//       }
		//     }
		//     else{
		//       if(c < (p1.x - l/2)){
		//         if(thetad==0){
		//           theta = 0;
		//         }
		//         else{
		//           theta = -(p1.x - c - l/2)/r;
		//         }
		//       }
		//       else{
		//         theta = 0;
		//       }
		//      }
		//
		//      fclose(fp);
		//
		//   }
		//
		//
		//
		//
		//   else if (ph == Phase::R && side == 1){
		//     FILE* fp1 = fopen("Test1.csv", "r");
		//     fscanf(fp1, "%lf", &thetad);
		//
		//     if(thetad == 0){
		//       theta = 0;
		//     }
		//     else{
		//       theta = theta0 + tau*(theta1 - theta0);
		//     }
		//
		//     fclose(fp1);
		//   }
		//   else if (ph == Phase::LR && side == 1){
		// 		FILE* fp1 = fopen("Test1.csv", "r");
		//     fscanf(fp1, "%lf", &thetad);
		// 		theta = thetad;
		//     fclose(fp1);
		//   }
		//   else if (ph == Phase::RL && side == 1) { //�ܐ您�낵
		//     FILE* fp1 = fopen("Test1.csv", "r");
		//     fscanf(fp1, "%lf", &thetad);
		//
		//     if(thetad == 0){
		//       theta = 0;
		//     }
		//     else{
		//       theta = -(p1.x - c1.x -l/2)/r;
		//     }
		//
		//     fclose(fp1);
		//   }
		//   else if(ph == Phase::L && side == 1){
		//     FILE* fp1 = fopen("Test1.csv", "r");
		//     fscanf(fp1, "%lf", &thetad);
		//
		//     if(((c0.x - p1.x) < -l/2) && ((c1.x - p1.x) > l/2)){
		//       if(c < (p1.x - l/2)){
		//         if(thetad==0){
		//           theta = 0;
		//         }
		//         else{
		//           theta = -(p1.x - c - l/2)/r;
		//         }
		//       }
		//       else if(c > (p1.x + l/2)){
		//         theta = (c - p1.x - l/2)/r;
		//       }
		//       else{
		//         theta = 0;
		//       }
		//     }
		//     else{
		//       if(c < (p1.x - l/2)){
		//         if(thetad==0){
		//           theta = 0;
		//         }
		//         else{
		//           theta = -(p1.x - c - l/2)/r;
		//         }
		//       }
		//       else{
		//         theta = 0;
		//       }
		//      }
		//
		//      fclose(fp1);
		//
		//   }

		//gait_change_smooth
		// if (param.swingProfile == SwingProfile::Heel_Toe){
		// 	real_t _2pi = 2.0*M_PI;
		// 	real_t tau = (t - t0) / h;
		// 	// real_t tau1 = (t - t0) / (td - t0);
		// 	// real_t tau2 = (t - tdd) / (t1 - tdd);
	  //   real_t c = c0.x + (c1.x - c0.x)*tau;
		//
		//
		//
		//
		// 	if(ph == Phase::L && side == 0){
		//     FILE* fp = fopen("Test.csv", "r");
		//     fscanf(fp, "%d", &thetad);
		//
		//     if(thetad == 0){
		//       theta = 0;
		//     }
		//     else{
		//       theta = theta0 + tau*(theta1 - theta0);
		//     }
		//
		//     fclose(fp);
		//   }
		//   else if (ph == Phase::RL && side == 0){
		// 		FILE* fp = fopen("Test.csv", "r");
		//     fscanf(fp, "%d", &thetad);
		//
		// 		if(thetad == 0){
 		// 		 theta = 0;
 		// 	 }
 		// 	 else{
 		// 		 theta_ds = (c0.x - p1.x -l/2)/r0;
 		// 		 theta = theta_ds + (theta0 - theta_ds)*tau;
 		// 	 }
		//
		//     fclose(fp);
		//   }
		//   else if (ph == Phase::LR && side == 0) { //�ܐ您�낵
		//     FILE* fp = fopen("Test.csv", "r");
		//     fscanf(fp, "%d", &thetad);
		//
		//     if(thetad == 0){
		//       theta = 0;
		//     }
		//     else{
		// 			theta_ds = (p1.x - c1.x -l/2)/r1;
		// 			theta = theta1 - (theta_ds + theta1)*tau;
		//     }
		//
		//     fclose(fp);
		//   }
		//   else if (ph == Phase::R && side == 0){
		//     FILE* fp = fopen("Test.csv", "r");
		//     fscanf(fp, "%d", &thetad);
		//
		//     if(((c0.x - p1.x) < -l/2) && ((c1.x - p1.x) > l/2)){
		//       if(c < (p1.x - l/2)){
		//         if(thetad==0){
		//           theta = 0;
		//         }
		//         else{
		//           theta = -(p1.x - c - l/2)/r1;
		//         }
		//       }
		//       else if(c > (p1.x + l/2)){
		//         theta = (c - p1.x - l/2)/r0;
		//       }
		//       else{
		//         theta = 0;
		//       }
		//     }
		//     else{
		//       if(c < (p1.x - l/2)){
		//         if(thetad==0){
		//           theta = 0;
		//         }
		//         else{
		//           theta = -(p1.x - c - l/2)/r1;
		//         }
		//       }
		//       else{
		//         theta = 0;
		//       }
		//      }
		//
		//      fclose(fp);
		//
		//   }
		//
		//
		//
		//
		//   else if (ph == Phase::R && side == 1){
		//     FILE* fp1 = fopen("Test1.csv", "r");
		//     fscanf(fp1, "%d", &thetad);
		//
		//     if(thetad == 0){
		//       theta = 0;
		//     }
		//     else{
		//       theta = theta0 + tau*(theta1 - theta0);
		//     }
		//
		//     fclose(fp1);
		//   }
		//   else if (ph == Phase::LR && side == 1){
		// 		FILE* fp1 = fopen("Test1.csv", "r");
		//     fscanf(fp1, "%d", &thetad);
		//
		// 		if(thetad == 0){
 		// 		 theta = 0;
 		// 	 }
 		// 	 else{
 		// 		 theta_ds = (c0.x - p1.x -l/2)/r0;
 		// 		 theta = theta_ds + (theta0 - theta_ds)*tau;
 		// 	 }
		//
		//     fclose(fp1);
		//   }
		//   else if (ph == Phase::RL && side == 1) { //�ܐ您�낵
		//     FILE* fp1 = fopen("Test1.csv", "r");
		//     fscanf(fp1, "%d", &thetad);
		//
		//     if(thetad == 0){
		//       theta = 0;
		//     }
		//     else{
		// 			theta_ds = (p1.x - c1.x -l/2)/r1;
		// 			theta = theta1 - (theta_ds + theta1)*tau;
		//     }
		//
		//     fclose(fp1);
		//   }
		//   else if(ph == Phase::L && side == 1){
		//     FILE* fp1 = fopen("Test1.csv", "r");
		//     fscanf(fp1, "%d", &thetad);
		//
		//     if(((c0.x - p1.x) < -l/2) && ((c1.x - p1.x) > l/2)){
		//       if(c < (p1.x - l/2)){
		//         if(thetad==0){
		//           theta = 0;
		//         }
		//         else{
		//           theta = -(p1.x - c - l/2)/r1;
		//         }
		//       }
		//       else if(c > (p1.x + l/2)){
		//         theta = (c - p1.x - l/2)/r0;
		//       }
		//       else{
		//         theta = 0;
		//       }
		//     }
		//     else{
		//       if(c < (p1.x - l/2)){
		//         if(thetad==0){
		//           theta = 0;
		//         }
		//         else{
		//           theta = -(p1.x - c - l/2)/r1;
		//         }
		//       }
		//       else{
		//         theta = 0;
		//       }
		//      }
		//
		//      fclose(fp1);
		//
		//   }

			//various_gait
			// if (param.swingProfile == SwingProfile::Heel_Toe){
			// 	real_t _2pi = 2.0*M_PI;
			// 	real_t tau = (t - t0) / h;
			// 	// real_t tau1 = (t - t0) / (td - t0);
			// 	// real_t tau2 = (t - tdd) / (t1 - tdd);
		  //   real_t c = c0.x + (c1.x - c0.x)*tau;
			//
			//
			//
			//
			// 	if(ph == Phase::L && side == 0){
			//     FILE* fp = fopen("Test.csv", "r");
			//     fscanf(fp, "%d", &thetad);
			//
			//     if(thetad == 0){
			//       theta = 0;
			//     }
			//     else{
			//       theta = theta0 + tau*(theta1 - theta0);
			//     }
			//
			//     fclose(fp);
			//   }
			//   else if (ph == Phase::RL && side == 0){
			// 		FILE* fp = fopen("Test.csv", "r");
			//     fscanf(fp, "%d", &thetad);
			//
			// 		if(thetad == 0){
	 		// 		 theta = 0;
	 		// 	 }
	 		// 	 else{
	 		// 		 theta_ds = (c0.x - p1.x -l0)/r0;
	 		// 		 theta = theta_ds + (theta0 - theta_ds)*tau;
	 		// 	 }
			//
			//     fclose(fp);
			//   }
			//   else if (ph == Phase::LR && side == 0) { //�ܐ您�낵
			//     FILE* fp = fopen("Test.csv", "r");
			//     fscanf(fp, "%d", &thetad);
			//
			//     if(thetad == 0){
			//       theta = 0;
			//     }
			//     else{
			// 			theta_ds = (p1.x - c1.x -l1)/r1;
			// 			theta = theta1 - (theta_ds + theta1)*tau;
			//     }
			//
			//     fclose(fp);
			//   }
			//   else if (ph == Phase::R && side == 0){
			//     FILE* fp = fopen("Test.csv", "r");
			//     fscanf(fp, "%d", &thetad);
			//
			//     if(((c0.x - p1.x) < -l1) && ((c1.x - p1.x) > l0)){
			//       if(c < (p1.x - l1)){
			//         if(thetad==0){
			//           theta = 0;
			//         }
			//         else{
			//           theta = -(p1.x - c - l1)/r1;
			//         }
			//       }
			//       else if(c > (p1.x + l0)){
			//         theta = (c - p1.x - l0)/r0;
			//       }
			//       else{
			//         theta = 0;
			//       }
			//     }
			//     else{
			//       if(c < (p1.x - l1)){
			//         if(thetad==0){
			//           theta = 0;
			//         }
			//         else{
			//           theta = -(p1.x - c - l1)/r1;
			//         }
			//       }
			//       else{
			//         theta = 0;
			//       }
			//      }
			//
			//      fclose(fp);
			//
			//   }
			//
			//
			//
			//
			//   else if (ph == Phase::R && side == 1){
			//     FILE* fp1 = fopen("Test1.csv", "r");
			//     fscanf(fp1, "%d", &thetad);
			//
			//     if(thetad == 0){
			//       theta = 0;
			//     }
			//     else{
			//       theta = theta0 + tau*(theta1 - theta0);
			//     }
			//
			//     fclose(fp1);
			//   }
			//   else if (ph == Phase::LR && side == 1){
			// 		FILE* fp1 = fopen("Test1.csv", "r");
			//     fscanf(fp1, "%d", &thetad);
			//
			// 		if(thetad == 0){
	 		// 		 theta = 0;
	 		// 	 }
	 		// 	 else{
	 		// 		 theta_ds = (c0.x - p1.x -l0)/r0;
	 		// 		 theta = theta_ds + (theta0 - theta_ds)*tau;
	 		// 	 }
			//
			//     fclose(fp1);
			//   }
			//   else if (ph == Phase::RL && side == 1) { //�ܐ您�낵
			//     FILE* fp1 = fopen("Test1.csv", "r");
			//     fscanf(fp1, "%d", &thetad);
			//
			//     if(thetad == 0){
			//       theta = 0;
			//     }
			//     else{
			// 			theta_ds = (p1.x - c1.x -l1)/r1;
			// 			theta = theta1 - (theta_ds + theta1)*tau;
			//     }
			//
			//     fclose(fp1);
			//   }
			//   else if(ph == Phase::L && side == 1){
			//     FILE* fp1 = fopen("Test1.csv", "r");
			//     fscanf(fp1, "%d", &thetad);
			//
			//     if(((c0.x - p1.x) < -l1) && ((c1.x - p1.x) > l0)){
			//       if(c < (p1.x - l1)){
			//         if(thetad==0){
			//           theta = 0;
			//         }
			//         else{
			//           theta = -(p1.x - c - l1)/r1;
			//         }
			//       }
			//       else if(c > (p1.x + l0)){
			//         theta = (c - p1.x - l0)/r0;
			//       }
			//       else{
			//         theta = 0;
			//       }
			//     }
			//     else{
			//       if(c < (p1.x - l1)){
			//         if(thetad==0){
			//           theta = 0;
			//         }
			//         else{
			//           theta = -(p1.x - c - l1)/r1;
			//         }
			//       }
			//       else{
			//         theta = 0;
			//       }
			//      }
			//
			//      fclose(fp1);
			//
			//   }


			//foot_shape_change
			if (param.swingProfile == SwingProfile::Heel_Toe){
				real_t _2pi = 2.0*M_PI;
				real_t tau = (t - t0) / h;
				// real_t tau1 = (t - t0) / (td - t0);
				// real_t tau2 = (t - tdd) / (t1 - tdd);
		    real_t c = c0.x + (c1.x - c0.x)*tau;




				if(ph == Phase::L && side == 0){
			    FILE* fp = fopen("Test.csv", "r");
			    fscanf(fp, "%d", &thetad);

			    if(thetad == 0){
			      theta = 0;
			    }
			    else{
			      theta = theta0 + tau*(theta1 - theta0);
			    }

			    fclose(fp);
			  }
			  else if (ph == Phase::RL && side == 0){
					FILE* fp = fopen("Test.csv", "r");
			    fscanf(fp, "%d", &thetad);

					if(thetad == 0){
	 				 theta = 0;
	 			 }
	 			 else{
	 				 theta_ds = (c0.x - (p1.x + dev) -l0)/r0;
	 				 theta = theta_ds + (theta0 - theta_ds)*tau;
	 			 }

			    fclose(fp);
			  }
			  else if (ph == Phase::LR && side == 0) { //�ܐ您�낵
			    FILE* fp = fopen("Test.csv", "r");
			    fscanf(fp, "%d", &thetad);

			    if(thetad == 0){
			      theta = 0;
			    }
			    else{
						theta_ds = ((p1.x + dev) - c1.x -l1)/r1;
						theta = theta1 - (theta_ds + theta1)*tau;
			    }

			    fclose(fp);
			  }
			  else if (ph == Phase::R && side == 0){
			    FILE* fp = fopen("Test.csv", "r");
			    fscanf(fp, "%d", &thetad);

			    if(((c0.x - (p1.x + dev)) < -l1) && ((c1.x - (p1.x + dev)) > l0)){
			      if(c < ((p1.x + dev)- l1)){
			        if(thetad==0){
			          theta = 0;
			        }
			        else{
			          theta = -((p1.x + dev) - c - l1)/r1;
			        }
			      }
			      else if(c > ((p1.x + dev) + l0)){
			        theta = (c - (p1.x + dev)- l0)/r0;
			      }
			      else{
			        theta = 0;
			      }
			    }
			    else{
			      if(c < ((p1.x + dev) - l1)){
			        if(thetad==0){
			          theta = 0;
			        }
			        else{
			          theta = -((p1.x + dev) - c - l1)/r1;
			        }
			      }
			      else{
			        theta = 0;
			      }
			     }

			     fclose(fp);

			  }




			  else if (ph == Phase::R && side == 1){
			    FILE* fp1 = fopen("Test1.csv", "r");
			    fscanf(fp1, "%d", &thetad);

			    if(thetad == 0){
			      theta = 0;
			    }
			    else{
			      theta = theta0 + tau*(theta1 - theta0);
			    }

			    fclose(fp1);
			  }
			  else if (ph == Phase::LR && side == 1){
					FILE* fp1 = fopen("Test1.csv", "r");
			    fscanf(fp1, "%d", &thetad);

					if(thetad == 0){
	 				 theta = 0;
	 			 }
	 			 else{
	 				 theta_ds = (c0.x - (p1.x + dev) -l0)/r0;
	 				 theta = theta_ds + (theta0 - theta_ds)*tau;
	 			 }

			    fclose(fp1);
			  }
			  else if (ph == Phase::RL && side == 1) { //�ܐ您�낵
			    FILE* fp1 = fopen("Test1.csv", "r");
			    fscanf(fp1, "%d", &thetad);

			    if(thetad == 0){
			      theta = 0;
			    }
			    else{
						theta_ds = ((p1.x + dev) - c1.x -l1)/r1;
						theta = theta1 - (theta_ds + theta1)*tau;
			    }

			    fclose(fp1);
			  }
			  else if(ph == Phase::L && side == 1){
			    FILE* fp1 = fopen("Test1.csv", "r");
			    fscanf(fp1, "%d", &thetad);

			    if(((c0.x - (p1.x + dev)) < -l1) && ((c1.x - (p1.x + dev)) > l0)){
			      if(c < ((p1.x + dev) - l1)){
			        if(thetad==0){
			          theta = 0;
			        }
			        else{
			          theta = -((p1.x + dev) - c - l1)/r1;
			        }
			      }
			      else if(c > ((p1.x + dev) + l0)){
			        theta = (c - (p1.x + dev) - l0)/r0;
			      }
			      else{
			        theta = 0;
			      }
			    }
			    else{
			      if(c < ((p1.x + dev) - l1)){
			        if(thetad==0){
			          theta = 0;
			        }
			        else{
			          theta = -((p1.x + dev) - c - l1)/r1;
			        }
			      }
			      else{
			        theta = 0;
			      }
			     }

			     fclose(fp1);

			  }

			//foot_shape_diffferent
			// if (param.swingProfile == SwingProfile::Heel_Toe){
			// 	real_t _2pi = 2.0*M_PI;
			// 	real_t tau = (t - t0) / h;
			// 	// real_t tau1 = (t - t0) / (td - t0);
			// 	// real_t tau2 = (t - tdd) / (t1 - tdd);
		  //   real_t c = c0.x + (c1.x - c0.x)*tau;
			//
			//
			//
			//
			// 	if(ph == Phase::L && side == 0){
			//     FILE* fp = fopen("Test.csv", "r");
			//     fscanf(fp, "%d", &thetad);
			//
			//     if(thetad == 0){
			//       theta = 0;
			//     }
			//     else{
			//       theta = theta0 + tau*(theta1 - theta0);
			//     }
			//
			//     fclose(fp);
			//   }
			//   else if (ph == Phase::RL && side == 0){
			// 		FILE* fp = fopen("Test.csv", "r");
			//     fscanf(fp, "%d", &thetad);
			//
			// 		if(thetad == 0){
	 		// 		 theta = 0;
	 		// 	 }
	 		// 	 else{
	 		// 		 theta = theta0;
	 		// 	 }
			//
			//     fclose(fp);
			//   }
			//   else if (ph == Phase::LR && side == 0) { //�ܐ您�낵
			//     FILE* fp = fopen("Test.csv", "r");
			//     fscanf(fp, "%d", &thetad);
			//
			//     if(thetad == 0){
			//       theta = 0;
			//     }
			//     else{
			// 			theta_ds = (p1.x - c1.x -l1)/r1;
			// 			theta = theta1 - (theta_ds + theta1)*tau;
			//     }
			//
			//     fclose(fp);
			//   }
			//   else if (ph == Phase::R && side == 0){
			//     FILE* fp = fopen("Test.csv", "r");
			//     fscanf(fp, "%d", &thetad);
			//
			//     if(((c0.x - p1.x) < -l1) && ((c1.x - p1.x) > (l0 + r0*theta0))){
			//       if(c < (p1.x - l1)){
			//         if(thetad==0){
			//           theta = 0;
			//         }
			//         else{
			//           theta = -(p1.x - c - l1)/r1;
			//         }
			//       }
			//       else if(c > (p1.x + l0) && c < (p1.x + l0 + r0*theta0)){
			//         theta = (c - p1.x - l0)/r0;
			//       }
			// 			else if(c > (p1.x + l0 + r0*theta0)){
			// 				theta = theta0;
			// 			}
			//       else{
			//         theta = 0;
			//       }
			//     }
			//     else{
			//       if(c < (p1.x - l1)){
			//         if(thetad==0){
			//           theta = 0;
			//         }
			//         else{
			//           theta = -(p1.x - c - l1)/r1;
			//         }
			//       }
			//       else{
			//         theta = 0;
			//       }
			//      }
			//
			//      fclose(fp);
			//
			//   }
			//
			//
			//
			//
			//   else if (ph == Phase::R && side == 1){
			//     FILE* fp1 = fopen("Test1.csv", "r");
			//     fscanf(fp1, "%d", &thetad);
			//
			//     if(thetad == 0){
			//       theta = 0;
			//     }
			//     else{
			//       theta = theta0 + tau*(theta1 - theta0);
			//     }
			//
			//     fclose(fp1);
			//   }
			//   else if (ph == Phase::LR && side == 1){
			// 		FILE* fp1 = fopen("Test1.csv", "r");
			//     fscanf(fp1, "%d", &thetad);
			//
			// 		if(thetad == 0){
	 		// 		 theta = 0;
	 		// 	 }
	 		// 	 else{
	 		// 		 theta = theta0;
	 		// 	 }
			//
			//     fclose(fp1);
			//   }
			//   else if (ph == Phase::RL && side == 1) { //�ܐ您�낵
			//     FILE* fp1 = fopen("Test1.csv", "r");
			//     fscanf(fp1, "%d", &thetad);
			//
			//     if(thetad == 0){
			//       theta = 0;
			//     }
			//     else{
			// 			theta_ds = (p1.x - c1.x -l1)/r1;
			// 			theta = theta1 - (theta_ds + theta1)*tau;
			//     }
			//
			//     fclose(fp1);
			//   }
			//   else if(ph == Phase::L && side == 1){
			//     FILE* fp1 = fopen("Test1.csv", "r");
			//     fscanf(fp1, "%d", &thetad);
			//
			//     if(((c0.x - p1.x) < -l1) && ((c1.x - p1.x) > (l0 + r0*theta0))){
			//       if(c < (p1.x - l1)){
			//         if(thetad==0){
			//           theta = 0;
			//         }
			//         else{
			//           theta = -(p1.x - c - l1)/r1;
			//         }
			//       }
			// 			else if(c > (p1.x + l0) && c < (p1.x + l0 + r0*theta0)){
			//         theta = (c - p1.x - l0)/r0;
			//       }
			// 			else if(c > (p1.x + l0 + r0*theta0)){
			// 				theta = theta0;
			// 			}
			//       else{
			//         theta = 0;
			//       }
			//     }
			//     else{
			//       if(c < (p1.x - l1)){
			//         if(thetad==0){
			//           theta = 0;
			//         }
			//         else{
			//           theta = -(p1.x - c - l1)/r1;
			//         }
			//       }
			//       else{
			//         theta = 0;
			//       }
			//      }
			//
			//      fclose(fp1);
			//
			//   }



	}
}

	  else {
	    theta=0;
	  }
	  return theta;
	}



	//CoP�ʒu
	vec3_t BipedLIP::CopPos(real_t t) {
		BipedLIPKey* key0 = (BipedLIPKey*)traj.GetSegment(t).first;
		BipedLIPKey* key1 = (BipedLIPKey*)traj.GetSegment(t).second;

		vec2_t ct;

		if (key1) {
			real_t dt = t - key0->var_time->val;
			real_t tau = key0->var_duration->val;
			vec2_t c0 = key0->var_cop_pos->val;
			vec2_t c1 = key1->var_cop_pos->val;

			ct = c0 + (c1 - c0)*(dt / tau);
		}
		else {
			ct = key0->var_cop_pos->val;
		}

		return vec3_t(ct.x, ct.y, 0.0);
	}

	//���̈ʒu(3.4.7 �d�S�̈��v����)
	vec3_t BipedLIP::TorsoPos(const vec3_t& pcom, const vec3_t& psup, const vec3_t& pswg) {
		// �R���p�X���f�����蓷�̂̈ʒu�����߂�
		real_t mt = param.torsoMass;
		real_t mf = param.footMass;
		vec3_t pt = ((mt + 2.0*mf)*pcom - mf * (psup + pswg)) / mt;
		return pt;
	}

	//------------------------------------------------------------------------------------------------

	void BipedLIP::CalcTrajectory() {
		real_t tf = traj.back()->tick->time;
		real_t dt = 0.01;

		trajectory.clear();
		for (real_t t = 0.0; t <= tf; t += dt) {
			TrajPoint tp;
			tp.t = t;
			tp.com_pos = ComPos(t);
			tp.foot_pos_t[0] = FootPos(t, 0);
			tp.foot_pos_r[0] = FootOri(t, 0);
			tp.foot_pos_t[1] = FootPos(t, 1);
			tp.foot_pos_r[1] = FootOri(t, 1);
			tp.torso_pos_t = TorsoPos(tp.com_pos, tp.foot_pos_t[0], tp.foot_pos_t[1]);
			tp.torso_pos_r = TorsoOri(t);
			tp.cop_pos = CopPos(t);

			trajectory.push_back(tp);
		}

		trajReady = true;
	}

	//------------------------------------------------------------------------------------------------

	void BipedLIP::Draw(Render::Canvas* canvas, Render::Config* conf) {
		TrajectoryNode::Draw(canvas, conf);

		if (!trajReady)
			CalcTrajectory();

		if (trajectory.empty())
			return;

		// com
		if (conf->Set(canvas, Render::Item::BipedCom, this)) {
			canvas->BeginLayer("biped_com", true);
			canvas->SetLineWidth(3.0f);
			canvas->SetLineColor("black");
			canvas->BeginPath();
			canvas->MoveTo(trajectory[0].com_pos);
			for (uint i = 1; i < trajectory.size(); i++) {
				canvas->LineTo(trajectory[i].com_pos);
			}
			canvas->EndPath();
			canvas->EndLayer();
		}

		// cop
		if (conf->Set(canvas, Render::Item::BipedCop, this)) {
			canvas->BeginLayer("biped_cop", true);
			canvas->SetLineWidth(3.0f);
			canvas->SetLineColor("magenta");
			canvas->BeginPath();
			canvas->MoveTo(trajectory[0].cop_pos);
			for (uint i = 1; i < trajectory.size(); i++) {
				canvas->LineTo(trajectory[i].cop_pos);
			}
			canvas->EndPath();
			canvas->EndLayer();
		}

		// torso
		if (conf->Set(canvas, Render::Item::BipedTorso, this)) {
			canvas->BeginLayer("biped_torso", true);
			canvas->SetLineWidth(2.0f);
			canvas->SetLineColor("blue");
			canvas->BeginPath();
			canvas->MoveTo(trajectory[0].torso_pos_t);
			for (uint i = 1; i < trajectory.size(); i++) {
				canvas->LineTo(trajectory[i].torso_pos_t);
			}
			canvas->EndPath();
			canvas->EndLayer();
		}


		// swing foot
		if (conf->Set(canvas, Render::Item::BipedFoot, this)) {
			canvas->BeginLayer("biped_foot", true);
			canvas->SetLineWidth(1.0f);
			canvas->BeginPath();
			canvas->MoveTo(trajectory[0].foot_pos_t[0]);
			for (uint i = 1; i < trajectory.size(); i++) {
				canvas->LineTo(trajectory[i].foot_pos_t[0]);
			}
			canvas->MoveTo(trajectory[0].foot_pos_t[1]);
			for (uint i = 1; i < trajectory.size(); i++) {
				canvas->LineTo(trajectory[i].foot_pos_t[1]);
			}
			canvas->EndPath();
			canvas->EndLayer();
		}
	}

	void BipedLIP::DrawSnapshot(real_t time, Render::Canvas* canvas, Render::Config* conf) {
		canvas->SetLineWidth(2.0f);
		canvas->BeginPath();
		canvas->MoveTo(ComPos(time));
		canvas->LineTo(FootPos(time, 0));
		canvas->MoveTo(ComPos(time));
		canvas->LineTo(FootPos(time, 1));
		canvas->EndPath();
	}

	void BipedLIP::Save() {
		//FILE* file = fopen("C:/Users/repor/OneDrive/�h�L�������g/DiMP_save//exa.csv", "w");
		FILE* file = fopen("C:/devel/DiMP/DiMP_save/exa.csv", "w");

		real_t dt = 0.0001;
		real_t tf = traj.back()->tick->time;
		for (real_t t = 0.0; t <= tf; t += dt) {

			vec3_t com_p = ComPos(t);
			vec3_t lfoot_p = FootPos(t, 1);
			vec3_t rfoot_p = FootPos(t, 0);
			vec3_t cop_p = CopPos(t);

			fprintf(file, "%3.4lf,", t);
			fprintf(file, "%3.4lf, %3.4lf, %3.4lf,", com_p.x, com_p.y, com_p.z);
			fprintf(file, "%3.4lf, %3.4lf, %3.4lf, %3.4lf, %3.4lf, %3.4lf,", lfoot_p.x, lfoot_p.y, lfoot_p.z, rfoot_p.x, rfoot_p.y, rfoot_p.z);
			fprintf(file, "%3.4lf, %3.4lf\n", cop_p.x, cop_p.y);
		}
		fclose(file);


		//FILE* fp = fopen("//LANDISK3/Public/�l�p(Personal)/kataoka/�����f�[�^/2018_10/10_25//Flat_0.30m_100step_data.csv", "w");
		////fprintf(fp, "�X�e�b�v, ����t, ������, �d�S�ʒuCoM_p_x, �d�S�ʒuCoM_p_x, �d�S�ʒuCoM_p_x, �d�S�ʒuCoM_p_x,");
		////fprintf(fp, "�ڒn�ʒuCoP_p_x, �ڒn�ʒuCoP_p_x\n");

		//real_t t = 0.0;
		//for (uint k = 0; k < graph->ticks.size(); k++) {
		//	BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[k]);

		//	if (k != (graph->ticks.size()) - 1) {
		//		fprintf(fp, "%d, %3.4lf, %3.4lf\n",
		//			k, t, key->var_duration->val);
		//		t += key->var_duration->val;
		//	}
		//	else {
		//		fprintf(fp, "%d, %3.4lf\n",
		//			k, t);
		//	}
		//}
		//fclose(fp);
	}


	void BipedLIP::Print()//���ʂɕ\�������֐�
	{
		real_t dt = 0.0001;
		real_t tf = traj.back()->tick->time;
		for (real_t t = 0.0; t <= tf; t += dt) {
			printf("%3.4lf,\n", t);
		}
	}


	//-------------------------------------------------------------------------------------------------

	// Constructors
	BipedLipCon::BipedLipCon(Solver* solver, int _tag, string _name, BipedLIPKey* _obj, real_t _scale) :
		Constraint(solver, 2, ID(_tag, _obj->node, _obj->tick, _name), _scale) {
		obj[0] = _obj;
		obj[1] = (BipedLIPKey*)_obj->next;
	}

	BipedLipConP::BipedLipConP(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
		BipedLipCon(solver, ConTag::BipedLipP, _name, _obj, _scale) {

		AddSLink(obj[1]->var_com_pos);
		AddSLink(obj[1]->var_cop_pos);
		AddSLink(obj[0]->var_com_pos);
		AddSLink(obj[0]->var_com_vel);
		AddSLink(obj[0]->var_cop_pos);
		AddC2Link(obj[0]->var_duration);
	}

	BipedLipConV::BipedLipConV(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
		BipedLipCon(solver, ConTag::BipedLipV, _name, _obj, _scale) {

		AddSLink(obj[1]->var_com_vel);
		AddSLink(obj[1]->var_cop_pos);
		AddSLink(obj[0]->var_com_pos);
		AddSLink(obj[0]->var_com_vel);
		AddSLink(obj[0]->var_cop_pos);
		AddC2Link(obj[0]->var_duration);
	}

	/*
	CoMConR::CoMConR(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, real_t _scale) :
	Constraint(solver, 1, ID(ConTag::BipedCoMR, _obj->node, _obj->tick, _name), _scale) {
	obj[0] = _obj;
	obj[1] = (BipedLIPKey*)_obj->next;
	idx = _idx;

	AddSLink(obj[0]->com_pos_r);
	AddSLink(obj[0]->com_vel_r);
	AddSLink(obj[0]->period   );
	AddSLink(obj[1]->com_pos_r);
	AddSLink(obj[1]->com_vel_r);
	}
	*/

	BipedComConP::BipedComConP(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
		Constraint(solver, 2, ID(ConTag::BipedComP, _obj->node, _obj->tick, _name), _scale) {

		obj = _obj;

		AddSLink(obj->var_com_pos);
		AddSLink(obj->var_torso_pos_t);
		AddSLink(obj->var_foot_pos_t[0]);
		AddSLink(obj->var_foot_pos_t[1]);
	}

	BipedComConV::BipedComConV(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
		Constraint(solver, 2, ID(ConTag::BipedComV, _obj->node, _obj->tick, _name), _scale) {

		obj = _obj;

		AddSLink(obj->var_com_vel);
		AddSLink(obj->var_torso_vel_t);
	}

	BipedFootConT::BipedFootConT(Solver* solver, string _name, BipedLIPKey* _obj, uint _side, real_t _scale) :
		Constraint(solver, 2, ID(ConTag::BipedFootRangeT, _obj->node, _obj->tick, _name), _scale) {

		obj = _obj;
		side = _side;

		AddM2Link(obj->var_foot_pos_t[side]);
		AddM2Link(obj->var_torso_pos_t);
		AddC2Link(obj->var_torso_pos_r);
	}

	BipedFootConR::BipedFootConR(Solver* solver, string _name, BipedLIPKey* _obj, uint _side, real_t _scale) :
		Constraint(solver, 1, ID(ConTag::BipedFootRangeR, _obj->node, _obj->tick, _name), _scale) {

		obj = _obj;
		side = _side;

		AddSLink(obj->var_foot_pos_r[side]);
		AddSLink(obj->var_torso_pos_r);
	}

	BipedCopCon::BipedCopCon(Solver* solver, string _name, BipedLIPKey* _obj, uint _side, real_t _scale) :
		Constraint(solver, 2, ID(ConTag::BipedCop, _obj->node, _obj->tick, _name), _scale) {

		obj = _obj;
		side = _side;

		AddM2Link(obj->var_cop_pos);
		AddM2Link(obj->var_foot_pos_t[side]);
		AddC2Link(obj->var_foot_pos_r[side]);
	}

	BipedTimeCon::BipedTimeCon(Solver* solver, string _name, BipedLIPKey* _obj, real_t _scale) :
		Constraint(solver, 1, ID(ConTag::BipedTime, _obj->node, _obj->tick, _name), _scale) {

		obj[0] = _obj;
		obj[1] = (BipedLIPKey*)_obj->next;

		AddSLink(obj[1]->var_time);
		AddSLink(obj[0]->var_time);
		AddSLink(obj[0]->var_duration);
	}

	//-------------------------------------------------------------------------------------------------

	void BipedLipCon::Prepare() {
		BipedLIP::Param& param = ((BipedLIP*)obj[0]->node)->param;

		T = param.T;
		T2 = T * T;
		tau = obj[0]->var_duration->val;
		tau2 = tau * tau;
		C = cosh(tau / T);
		S = sinh(tau / T);
		p0 = obj[0]->var_com_pos->val;
		v0 = obj[0]->var_com_vel->val;
		c0 = obj[0]->var_cop_pos->val;
		p1 = obj[1]->var_com_pos->val;
		v1 = obj[1]->var_com_vel->val;
		c1 = obj[1]->var_cop_pos->val;
	}

	void BipedLipConP::CalcCoef() {//�d�S�ʒu(����-�E��)���Δ���
		Prepare();

		((SLink*)links[0])->SetCoef(1.0);
		((SLink*)links[1])->SetCoef(-1.0 + (T / tau)*S);
		((SLink*)links[2])->SetCoef(-C);
		((SLink*)links[3])->SetCoef(-T * S);
		((SLink*)links[4])->SetCoef(C - (T / tau)*S);
		((C2Link*)links[5])->SetCoef(-(S / T)*(p0 - c0) - C * (v0 - (c1 - c0) / tau) - (T*S)*(c1 - c0) / tau2);
	}

	void BipedLipConV::CalcCoef() {//�d�S���x(����-�E��)���Δ���
		Prepare();

		((SLink*)links[0])->SetCoef(1.0);
		((SLink*)links[1])->SetCoef((C - 1.0) / tau);
		((SLink*)links[2])->SetCoef(-S / T);
		((SLink*)links[3])->SetCoef(-C);
		((SLink*)links[4])->SetCoef((1.0 - C) / tau + S / T);
		((C2Link*)links[5])->SetCoef(-(C / T2)*(p0 - c0) - (S / T)*(v0 - (c1 - c0) / tau) + (1.0 - C)*(c1 - c0) / tau2);
	}

	/*
	void CoMConR::CalcCoef() {
	q0 = obj[0]->com_pos_r->val;
	w0 = obj[0]->com_vel_r->val;
	q1 = obj[1]->com_pos_r->val;
	w1 = obj[1]->com_vel_r->val;
	tau = obj[0]->period->val;
	tau2 = tau*tau;
	tau3 = tau*tau2;

	if (idx == 0) {
	((SLink*)links[0])->SetCoef(-6.0 / tau2);
	((SLink*)links[1])->SetCoef(-4.0 / tau);
	((SLink*)links[2])->SetCoef(-12.0*(q1 - q0) / tau3 + (4.0*w0 + 2.0*w1) / tau2);
	((SLink*)links[3])->SetCoef(6.0 / tau2);
	((SLink*)links[4])->SetCoef(-2.0 / tau);
	}
	else {
	((SLink*)links[0])->SetCoef(6.0 / tau2);
	((SLink*)links[1])->SetCoef(2.0 / tau);
	((SLink*)links[2])->SetCoef(12.0*(q1 - q0) / tau3 - (2.0*w0 + 4.0*w1) / tau2);
	((SLink*)links[3])->SetCoef(-6.0 / tau2);
	((SLink*)links[4])->SetCoef(4.0 / tau);
	}
	}
	*/

	void BipedComConP::CalcCoef() {
		BipedLIP::Param& param = ((BipedLIP*)obj->node)->param;

		real_t mt = param.torsoMass;
		real_t mf = param.footMass;

		((SLink*)links[0])->SetCoef(mt + 2.0*mf);
		((SLink*)links[1])->SetCoef(-mt);
		((SLink*)links[2])->SetCoef(-mf);
		((SLink*)links[3])->SetCoef(-mf);

	}

	void BipedComConV::CalcCoef() {
		BipedLIP::Param& param = ((BipedLIP*)obj->node)->param;

		real_t mt = param.torsoMass;
		real_t mf = param.footMass;

		((SLink*)links[0])->SetCoef(mt + 2.0*mf);
		((SLink*)links[1])->SetCoef(-mt);
	}

	void BipedFootConT::CalcCoef() {
		pf = obj->var_foot_pos_t[side]->val;
		pt = obj->var_torso_pos_t->val;
		thetat = obj->var_torso_pos_r->val;
		R = mat2_t::Rot(thetat);
		dR = mat2_t::Rot(thetat + pi / 2);

		((M2Link*)links[0])->SetCoef(R.trans());
		((M2Link*)links[1])->SetCoef(-R.trans());
		((C2Link*)links[2])->SetCoef(dR.trans()*(pf - pt));
	}

	void BipedFootConR::CalcCoef() {
		thetaf = obj->var_foot_pos_r[side]->val;
		thetat = obj->var_torso_pos_r->val;

		((SLink*)links[0])->SetCoef(1.0);
		((SLink*)links[1])->SetCoef(-1.0);
	}

	void BipedCopCon::CalcCoef() {
		pc = obj->var_cop_pos->val;
		pf = obj->var_foot_pos_t[side]->val;
		thetaf = obj->var_foot_pos_r[side]->val;
		R = mat2_t::Rot(thetaf);
		dR = mat2_t::Rot(thetaf + pi / 2);

		((M2Link*)links[0])->SetCoef(R.trans());
		((M2Link*)links[1])->SetCoef(-R.trans());
		((C2Link*)links[2])->SetCoef(dR.trans()*(pc - pf));
	}

	void BipedTimeCon::CalcCoef() {
		((SLink*)links[0])->SetCoef(1.0);
		((SLink*)links[1])->SetCoef(-1.0);
		((SLink*)links[2])->SetCoef(-1.0);
	}

	//-------------------------------------------------------------------------------------------------

	void BipedLipConP::CalcDeviation() {
		vec2_t y2 = p1 - c1 - C * (p0 - c0) - (T*S)*(v0 - (c1 - c0) / tau);
		y[0] = y2[0];
		y[1] = y2[1];
	}

	void BipedLipConV::CalcDeviation() {
		vec2_t y2 = v1 - (c1 - c0) / tau - (S / T)*(p0 - c0) - C * (v0 - (c1 - c0) / tau);
		y[0] = y2[0];
		y[1] = y2[1];
	}

	/*
	void CoMConR::CalcDeviation() {
	real_t s;
	if (idx == 0)
	s =  6.0*(q1 - q0) / tau2 - (4.0*w0 + 2.0*w1) / tau;
	else s = -6.0*(q1 - q0) / tau2 + (2.0*w0 + 4.0*w1) / tau;
	on_lower = (s < _min);
	on_upper = (s > _max);
	active = on_lower | on_upper;
	if (on_lower) y[0] = (s - _min);
	if (on_upper) y[0] = (s - _max);
	}
	*/

	void BipedFootConT::CalcDeviation() {
		vec2_t s = R.trans() * (pf - pt);

		active = false;
		for (uint i = 0; i < 2; i++) {
			on_lower[i] = (s[i] < _min[i]);
			on_upper[i] = (s[i] > _max[i]);
			if (on_lower[i]) {
				y[i] = (s[i] - _min[i]);
				active = true;
			}
			if (on_upper[i]) {
				y[i] = (s[i] - _max[i]);
				active = true;
			}
		}
	}

	void BipedFootConR::CalcDeviation() {
		real_t s = thetaf - thetat;
		on_lower = (s < _min);
		on_upper = (s > _max);
		active = on_lower | on_upper;
		if (on_lower) y[0] = (s - _min);
		if (on_upper) y[0] = (s - _max);
	}

	void BipedCopCon::CalcDeviation() {
		vec2_t s = R.trans() * (pc - pf);

		active = false;
		for (uint i = 0; i < 2; i++) {
			on_lower[i] = (s[i] < _min[i]);
			on_upper[i] = (s[i] > _max[i]);
			if (on_lower[i]) {
				y[i] = (s[i] - _min[i]);
				active = true;
			}
			if (on_upper[i]) {
				y[i] = (s[i] - _max[i]);
				active = true;
			}
		}
	}

	//-------------------------------------------------------------------------------------------------

	/*
	void CoMConR::Project(real_t& l, uint k) {
	if (on_upper &&  l > 0.0) l = 0.0;
	if (on_lower &&  l < 0.0) l = 0.0;
	if (!on_upper && !on_lower) l = 0.0;
	}
	*/

	void BipedFootConT::Project(real_t& l, uint k) {
		if (on_upper[k] && l > 0.0) l = 0.0;
		if (on_lower[k] && l < 0.0) l = 0.0;
		if (!on_upper[k] && !on_lower[k]) l = 0.0;
	}

	void BipedFootConR::Project(real_t& l, uint k) {
		if (on_upper &&  l > 0.0) l = 0.0;
		if (on_lower &&  l < 0.0) l = 0.0;
		if (!on_upper && !on_lower) l = 0.0;
	}

	void BipedCopCon::Project(real_t& l, uint k) {
		if (on_upper[k] && l > 0.0) l = 0.0;
		if (on_lower[k] && l < 0.0) l = 0.0;
		if (!on_upper[k] && !on_lower[k]) l = 0.0;
	}

}
