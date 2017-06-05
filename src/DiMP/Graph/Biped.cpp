#include <DiMP2/Biped.h>
#include <DiMP2/Graph.h>
#include <DiMP2/Solver.h>
#include <DiMP2/DrawConfig.h>
#include <DiMP2/DrawCanvas.h>
#include<stdio.h>


namespace DiMP2 {
	;

	//-------------------------------------------------------------------------------------------------
	// BipedLIPKey

	BipedLIPKey::BipedLIPKey() {
		phase = BipedLIP::Phase::Right;
	}

	//�ϐ���ǉ�����֐�
	void BipedLIPKey::AddVar(Solver* solver) {
		BipedLIP* obj = (BipedLIP*)node;

		com_pos_t[0] = new SVar(solver, ID(VarTag::BipedCoMTP, node, tick, name + "_com_tp0"), node->graph->scale.pos_t);//x������CoM�ʒu
		com_pos_t[1] = new SVar(solver, ID(VarTag::BipedCoMTP, node, tick, name + "_com_tp1"), node->graph->scale.pos_t);//y������CoM�ʒu
		com_vel_t[0] = new SVar(solver, ID(VarTag::BipedCoMTV, node, tick, name + "_com_tv0"), node->graph->scale.vel_t);//x������CoM���x
		com_vel_t[1] = new SVar(solver, ID(VarTag::BipedCoMTV, node, tick, name + "_com_tv1"), node->graph->scale.vel_t);//y������CoM���x

		com_pos_r = new SVar(solver, ID(VarTag::BipedCoMRP, node, tick, name + "_com_rp"), node->graph->scale.pos_r);
		com_vel_r = new SVar(solver, ID(VarTag::BipedCoMRV, node, tick, name + "_com_rv"), node->graph->scale.vel_r);

		// k���ڂ�cop�͗V�r�O���̏I�_�Ƃ��ĕK�v
		cop_pos_t[0] = new SVar(solver, ID(VarTag::BipedCoPT, node, tick, name + "_cop_t0"), node->graph->scale.pos_t);
		cop_pos_t[1] = new SVar(solver, ID(VarTag::BipedCoPT, node, tick, name + "_cop_t1"), node->graph->scale.pos_t);
		cop_pos_r = new SVar(solver, ID(VarTag::BipedCoPR, node, tick, name + "_cop_r"), node->graph->scale.pos_r);

		// k���ڂ̗V�r�n�_
		if (!prev) {
			swg_pos_t[0] = new SVar(solver, ID(VarTag::BipedCoPT, node, tick, name + "_swg_t0"), node->graph->scale.pos_t);
			swg_pos_t[1] = new SVar(solver, ID(VarTag::BipedCoPT, node, tick, name + "_swg_t1"), node->graph->scale.pos_t);
			swg_pos_r = new SVar(solver, ID(VarTag::BipedCoPR, node, tick, name + "_swg_r"), node->graph->scale.pos_r);
		}
		//�ڒn����
		if (next) {
			period = new SVar(solver, ID(VarTag::BipedPeriod, node, tick, name + "_period"), node->graph->scale.time);
		}
	}

	//�S��������ǉ�����֐�
	void BipedLIPKey::AddCon(Solver* solver) {
		BipedLIPKey* nextObj = (BipedLIPKey*)next;

		if (next) {
			con_com_tp[0] = new CoMConTP(solver, name + "_com_tpx", this, 0, node->graph->scale.pos_t);//x������CoM�ʒu
			con_com_tp[1] = new CoMConTP(solver, name + "_com_tpy", this, 1, node->graph->scale.pos_t);//y������CoM�ʒu
			con_com_tv[0] = new CoMConTV(solver, name + "_com_tvx", this, 0, node->graph->scale.vel_t);//x������CoM���x
			con_com_tv[1] = new CoMConTV(solver, name + "_com_tvy", this, 1, node->graph->scale.vel_t);//y������CoM���x

			con_com_r[0] = new CoMConR(solver, name + "_com_r0", this, 0, node->graph->scale.acc_r);
			con_com_r[1] = new CoMConR(solver, name + "_com_r1", this, 1, node->graph->scale.acc_r);

			con_cop_t[0][0] = new CoPConT(solver, name + "_cop_t0x", this, 0, 0, node->graph->scale.pos_t);//t0��x������CoP�ʒu
			con_cop_t[0][1] = new CoPConT(solver, name + "_cop_t0y", this, 0, 1, node->graph->scale.pos_t);//t0��y������CoP�ʒu
			con_cop_t[1][0] = new CoPConT(solver, name + "_cop_t1x", this, 1, 0, node->graph->scale.pos_t);//t1��x������CoP�ʒu
			con_cop_t[1][1] = new CoPConT(solver, name + "_cop_t1y", this, 1, 1, node->graph->scale.pos_t);//t1��y������CoP�ʒu

			con_cop_r[0] = new CoPConR(solver, name + "_cop_r0", this, 0, node->graph->scale.pos_r);
			con_cop_r[1] = new CoPConR(solver, name + "_cop_r1", this, 1, node->graph->scale.pos_r);

			con_period = new RangeConS(solver, ID(ConTag::BipedPeriod, node, tick, name + "_period"), period, node->graph->scale.time);//�ڒn����
		}
	}

	void BipedLIPKey::Prepare() {
		// ���s�����ϐ��̒l�������ɔ��f
		if (!prev) {
			tick->time = 0.0;
		}
		else {
			BipedLIPKey* prevObj = (BipedLIPKey*)prev;
			tick->time = prevObj->tick->time + prevObj->period->val;
		}
	}

	void BipedLIPKey::Draw(DrawCanvas* canvas, DrawConfig* conf) {
		const float l = 0.1f;

		Vec3f p;
		float theta;

		canvas->SetPointSize(8.0f);
		canvas->SetLineWidth(1.0f);

		// com
		p = Vec3f((float)com_pos_t[0]->val, (float)com_pos_t[1]->val, (float)((BipedLIP*)node)->param.heightCoM);
		theta = (float)com_pos_r->val;
		canvas->Point(p);
		//		canvas->Line(p, p + Vec3f(l*cos(theta), l*sin(theta), 0.0f));

		// cop
		if (next) {
			p = Vec3f((float)cop_pos_t[0]->val, (float)cop_pos_t[1]->val, 0.0f);
			theta = (float)cop_pos_r->val;
			canvas->Point(p);
			canvas->Line(p, p + Vec3f(l*cos(theta), l*sin(theta), 0.0f));
		}
	}

	//-------------------------------------------------------------------------------------------------
	// BipedLIP

	BipedLIP::Param::Param() {
		gravity = 9.8;//�d�͉����x
		heightCoM = 0.55;//�d�S����
		torsoMassRatio = 0.5;//���̎��ʂ̊���
		legMassRatio = 0.5;//�r���ʂ̊���

		swingProfile = SwingProfile::Cycloid;//�V�r�O���̎��
		swingHeight[0] = 0.04;///< 0: �V�r�̍ő卂��
		swingHeight[1] = 0.01;///< 1: �ڒn�O�̍ŏ����� for wedge only
		stepPeriodMin = 0.1;//�ŏ��̕Б��x�����̊���
		stepPeriodMax = 0.5;//�ő�̕Б��x�����̊���

							//�����\��CoM��CoP�̑��΋���
		supportPosMin[0] = vec2_t(-0.3, -0.2);
		supportPosMax[0] = vec2_t(0.3, -0.05);
		supportPosMin[1] = vec2_t(-0.3, 0.05);
		supportPosMax[1] = vec2_t(0.3, 0.2);

		supportOriMin[0] = -Rad(30.0);
		supportOriMax[0] = Rad(30.0);
		supportOriMin[1] = -Rad(30.0);
		supportOriMax[1] = Rad(30.0);
		angAccMax = 0.3;
		turnMax = 0.5;
	}

	/// �o�R�_
	//BipedLIP�N���X�̍\����Waypoint�̃����o�֐�Waypoint()
	BipedLIP::Waypoint::Waypoint() {
		k = 0;
		com_pos_r = 0.0;
		com_vel_r = 0.0;
		cop_pos_r = 0.0;
		swg_pos_r = 0.0;

		fix_com_pos_t = false;
		fix_com_vel_t = false;

		fix_com_pos_r = false;
		fix_com_vel_r = false;
		fix_cop_pos_t = false;
		fix_cop_pos_r = false;
		fix_swg_pos_t = false;
		fix_swg_pos_r = false;
	}

	//BipedLIP�N���X�̍\����TrajPoint�̃����o�֐�Trajpoint()
	BipedLIP::TrajPoint::TrajPoint() {
		t = 0.0;
		ori_com = 0.0;
		ori_sup = 0.0;
		ori_swg = 0.0;
	}

	BipedLIP::BipedLIP(Graph* g, string n) :TrajectoryNode(g, n) {
		type = Type::Object;
		graph->bipeds.Add(this);
	}

	BipedLIP::~BipedLIP() {
		graph->bipeds.Remove(this);
	}

	void BipedLIP::CalcSimple() {
		// �����݂��ĉ�͓I�ɋO�����o�����@
		real_t tau = (param.stepPeriodMin + param.stepPeriodMax) / 2.0;
		real_t C = cosh(tau / param.T);
		real_t S = sinh(tau / param.T);
		mat2_t A;
		vec2_t B;
		A[0][0] = C;         A[0][1] = S*param.T;
		A[1][0] = S / param.T; A[1][1] = C;
		B[0] = 1 - C;
		B[1] = -S / param.T;

		mat2_t A2 = A*A, A3 = A2*A, A4 = A3*A;
		vec2_t AB = A*B, A2B = A2*B, A3B = A3*B;

		mat2_t H;
		H.col(0) = A2B;
		H.col(1) = AB;

		vec2_t d;
		vec2_t U;
		vec2_t x[5];
		real_t u[4];
		for (int dir = 0; dir <= 1; dir++) {
			d = x[4] - A4*x[0] - A3B*u[0] - B*u[3];

			U = H.inv()*d;
			u[1] = U[0];
			u[2] = U[1];
		}

	}

	void BipedLIP::Init() {
		TrajectoryNode::Init();

		param.T = sqrt(param.heightCoM / param.gravity);//���萔T=��h/g

		real_t periodAve = (param.stepPeriodMin + param.stepPeriodMax) / 2.0;//���s�����̕���

		for (uint k = 0; k < graph->ticks.size(); k++) {
			BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[k]);

			if (key->next) {
				// �����̏����l�͉����Ə���̒��Ԓl
				key->phase = phase[k];
				key->period->val = periodAve;
				key->con_period->_min = param.stepPeriodMin;
				key->con_period->_max = param.stepPeriodMax;

				//�X�e�b�v�n�_�ƏI�_�̊p�����x����
				key->con_com_r[0]->_min = -param.angAccMax;
				key->con_com_r[0]->_max = param.angAccMax;
				key->con_com_r[1]->_min = -param.angAccMax;
				key->con_com_r[1]->_max = param.angAccMax;

				//�����\��CoM��CoP�̑��΋���
				for (int i = 0; i < 2; i++) {
					key->con_cop_t[i][0]->_min = param.supportPosMin[key->phase].x;//x�����̍ŏ����΋���
					key->con_cop_t[i][0]->_max = param.supportPosMax[key->phase].x;//x�����̍ŏ����΋���

					key->con_cop_t[i][1]->_min = param.supportPosMin[key->phase].y;//y�����̍ŏ����΋���
					key->con_cop_t[i][1]->_max = param.supportPosMax[key->phase].y;//y�����̍ŏ����΋���

					key->con_cop_r[i]->_min = param.supportOriMin[key->phase];
					key->con_cop_r[i]->_max = param.supportOriMax[key->phase];
				}
			}
		}

		// �d�S�ʒu�C�d�S���x�C���n�ʒu�̏����l���o�R�_�����ԃX�v���C���Ȑ�(���炩�ȕ�ԋȐ�)�ŗ^����
		Curve2d curve_pos;
		Curved  curve_ori;
		curve_pos.SetType(Interpolate::Cubic);//�R����ԂŌ���
		curve_ori.SetType(Interpolate::Cubic);
		uint idx = 0;
		for (uint i = 0; i < waypoints.size(); i++) {
			Waypoint& wp = waypoints[i];
			if (!wp.fix_com_pos_t) continue;//continue���˃u���b�N���̏������΂��C�u���b�N�̐擪�ʒu�ɖ߂��Ď��̏����𑱂���
			curve_pos.AddPoint(wp.k * periodAve);
			curve_ori.AddPoint(wp.k * periodAve);

			curve_pos.SetPos(idx, wp.com_pos_t);
			curve_pos.SetVel(idx, wp.com_vel_t);

			curve_ori.SetPos(idx, wp.com_pos_r);
			curve_ori.SetVel(idx, wp.com_vel_r);
			idx++;
		}
		for (uint k = 0; k < graph->ticks.size(); k++) {
			BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[k]);

			vec2_t p = curve_pos.CalcPos(k * periodAve);//CalPos�ˈʒu���v�Z����֐�
			vec2_t v = curve_pos.CalcVel(k * periodAve);//CalVel�ˑ��x���v�Z����֐�

			real_t a = curve_ori.CalcPos(k * periodAve);
			real_t r = curve_ori.CalcVel(k * periodAve);

			key->com_pos_t[0]->val = p[0];//t0��CoM�ʒu
			key->com_pos_t[1]->val = p[1];//t1��CoM�ʒu
			key->com_vel_t[0]->val = v[0];//t0��CoM���x
			key->com_vel_t[1]->val = v[1];//t1��CoM���x

			key->com_pos_r->val = a;
			key->com_vel_r->val = r;
			if (key->next) {
				p = curve_pos.CalcPos((k + 0.5) * periodAve);
				key->cop_pos_t[0]->val = p[0];
				key->cop_pos_t[1]->val = p[1];
			}
		}

		// �o�R�_��̕ϐ����Œ�
		for (uint i = 0; i < waypoints.size(); i++) {
			Waypoint& wp = waypoints[i];
			BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[wp.k]);

			if (wp.fix_com_pos_t) {
				key->com_pos_t[0]->val = wp.com_pos_t[0];
				key->com_pos_t[1]->val = wp.com_pos_t[1];
				key->com_pos_t[0]->locked = true;
				key->com_pos_t[1]->locked = true;
			}

			if (wp.fix_com_vel_t) {
				key->com_vel_t[0]->val = wp.com_vel_t[0];
				key->com_vel_t[1]->val = wp.com_vel_t[1];
				key->com_vel_t[0]->locked = true;
				key->com_vel_t[1]->locked = true;
			}

			if (wp.fix_com_pos_r) {
				key->com_pos_r->val = wp.com_pos_r;
				key->com_pos_r->locked = true;
			}

			if (wp.fix_com_vel_r) {
				key->com_vel_r->val = wp.com_vel_r;
				key->com_vel_r->locked = true;
			}

			if (wp.fix_cop_pos_t) {
				key->cop_pos_t[0]->val = wp.cop_pos_t[0];
				key->cop_pos_t[1]->val = wp.cop_pos_t[1];
				key->cop_pos_t[0]->locked = true;
				key->cop_pos_t[1]->locked = true;
			}

			if (wp.fix_cop_pos_r) {
				key->cop_pos_r->val = wp.cop_pos_r;
				key->cop_pos_r->locked = true;
			}

			if (wp.fix_swg_pos_t && !key->prev) {
				key->swg_pos_t[0]->val = wp.swg_pos_t[0];
				key->swg_pos_t[1]->val = wp.swg_pos_t[1];
				key->swg_pos_t[0]->locked = true;
				key->swg_pos_t[1]->locked = true;
			}

			if (wp.fix_swg_pos_r && !key->prev) {
				key->swg_pos_r->val = wp.swg_pos_r;
				key->swg_pos_r->locked = true;
			}
		}
	}

	void BipedLIP::Prepare() {
		TrajectoryNode::Prepare();
		trajReady = false;
	}

	int BipedLIP::Phase(real_t t) {
		return ((BipedLIPKey*)traj.GetSegment(t).first)->phase;
	}

	//�d�S�ʒu
	vec3_t BipedLIP::CoMPos(real_t t) {
		BipedLIPKey* key = (BipedLIPKey*)traj.GetSegment(t).first;

		vec3_t p;
		if (key->next) {
			real_t l = 0.1;//���̒��S�������E�ܐ�܂ł̋���
			real_t tau_ssp = (key->period->val)*0.9;//���s����(SSP)
			real_t tau_dsp = (key->period->val)*0.1;//���s����(DSP)

			real_t T = param.T;
			real_t t0 = key->tick->time;
			real_t x0_ssp = key->com_pos_t[0]->val;
			real_t y0_ssp = key->com_pos_t[1]->val;
			real_t vx0_ssp = key->com_vel_t[0]->val;
			real_t vy0_ssp = key->com_vel_t[1]->val;
			real_t px = key->cop_pos_t[0]->val;
			real_t py = key->cop_pos_t[1]->val;

			real_t x0_dsp = px + l + (x0_ssp - px + l)*cosh(+tau_ssp / T) + T*(vx0_ssp);
			real_t y0_dsp = py + (y0_ssp - py)*cosh(tau_ssp / T) + T*vy0_ssp*sinh(tau_ssp / T);
			real_t vx0_dsp = 2 * l / tau_ssp + (x0_ssp - px + l) / T*sinh(tau_ssp / T) + (vx0_ssp - (2 * l / tau_ssp))*cosh(tau_ssp / T);
			real_t vy0_dsp = (y0_ssp - py) / T*sinh(tau_ssp / T) + vy0_ssp*cosh(tau_ssp / T);

			real_t dx = key->next->cop_pos_t[0]->val - key->cop_pos_t[0]->val - 2 * l;
			real_t dy = key->next->cop_pos_t[1]->val - key->cop_pos_t[1]->val;

			if (t0<t && t<(t0 + tau_ssp)) {
				p.x = (2 * l / tau_ssp)*(t - t0) + (px - l) + (x0_ssp - px + l)*cosh((t - t0) / T) + T*(vx0_ssp - (2 * l / tau_ssp))*sinh((t - t0) / T);
				p.y = py + (y0_ssp - py) * cosh((t - t0) / T) + (vy0_ssp*T) * sinh((t - t0) / T);
				p.z = param.heightCoM;
			}
			else {
				p.x = (dx / tau_dsp)*(t - (t0 + tau_ssp)) + (px + l) + (x0_dsp - px - l)*cosh((t - (t0 + tau_ssp)) / T) + T*(vx0_dsp - (dx / tau_dsp))*sinh((t - (t0 + tau_ssp)) / T);
				p.y = (dy / tau_dsp)*(t - (t0 + tau_ssp)) + py + (y0_dsp - py)*cosh((t - (t0 + tau_ssp)) / T) + T*(vy0_dsp - (dy / tau_dsp))*sinh((t - (t0 + tau_ssp)) / T);
				p.z = param.heightCoM;
			}
		}
		else {
			p.x = key->com_pos_t[0]->val;
			p.y = key->com_pos_t[1]->val;
			p.z = param.heightCoM;
		}

		return p;
	}

	//�d�S���x
	vec3_t BipedLIP::CoMVel(real_t t) {
		BipedLIPKey* key = (BipedLIPKey*)traj.GetSegment(t).first;

		vec3_t v;
		if (key->next) {
			real_t l = 0.1;//���̒��S�������E�ܐ�܂ł̋���
			real_t tau_ssp = (key->period->val)*0.9;//���s����(SSP)
			real_t tau_dsp = (key->period->val)*0.1;//���s����(DSP)

			real_t T = param.T;
			real_t t0 = key->tick->time;
			real_t x0_ssp = key->com_pos_t[0]->val;
			real_t y0_ssp = key->com_pos_t[1]->val;
			real_t vx0_ssp = key->com_vel_t[0]->val;
			real_t vy0_ssp = key->com_vel_t[1]->val;
			real_t px = key->cop_pos_t[0]->val;
			real_t py = key->cop_pos_t[1]->val;

			real_t x0_dsp = px + l + (x0_ssp - px + l)*cosh(+tau_ssp / T) + T*(vx0_ssp);
			real_t y0_dsp = py + (y0_ssp - py)*cosh(tau_ssp / T) + T*vy0_ssp*sinh(tau_ssp / T);
			real_t vx0_dsp = 2 * l / tau_ssp + (x0_ssp - px + l) / T*sinh(tau_ssp / T) + (vx0_ssp - (2 * l / tau_ssp))*cosh(tau_ssp / T);
			real_t vy0_dsp = (y0_ssp - py) / T*sinh(tau_ssp / T) + vy0_ssp*cosh(tau_ssp / T);

			real_t dx = key->next->cop_pos_t[0]->val - key->cop_pos_t[0]->val - 2 * l;
			real_t dy = key->next->cop_pos_t[1]->val - key->cop_pos_t[1]->val;

			if (t0<t && t<(t0 + tau_ssp)) {
				v.x = (2 * l / tau_ssp) + (x0_ssp - px + l) / T*sinh((t - t0) / T) + (vx0_ssp - (2 * l / tau_ssp))*cosh((t - t0) / T);
				v.y = ((y0_ssp - py) / T) * sinh((t - t0) / T) + vy0_ssp* cosh((t - t0) / T);
				v.z = 0.0;
			}
			else {
				v.x = (dx / tau_dsp) + (x0_dsp - px - l) / T*sinh((t - (t0 + tau_ssp)) / T) + (vx0_dsp - (dx / tau_dsp))*cosh((t - (t0 + tau_ssp)) / T);
				v.y = (dy / tau_dsp) + (y0_dsp - py) / T*sinh((t - (t0 + tau_ssp)) / T) + (vy0_dsp - (dy / tau_dsp))*cosh((t - (t0 + tau_ssp)) / T);
				v.z = 0.0;
			}
		else {
			v.x = key->com_vel_t[0]->val;
			v.y = key->com_vel_t[1]->val;
			v.z = 0.0;
		}

		return v;
		}

		/*	void BipedLIP::Save() {
		FILE* file = fopen("CoMVel.csv", "w");
		fprintf(file, " time, vel_com_x\n");


		real_t t = 0.0;
		for (uint k = 0; k < graph->ticks.size(); k++) {//graph->ticks.size�������ɑ���
		BipedLIPKey* key = (BipedLIPKey*)traj.GetSegment(t).first;



		fprintf(file, " %3.3lf, %3.3lf\n",t,key->period->val);
		t += key->period->val;



		}

		fclose(file);
		}*/

		//�d�S�����x
		vec3_t BipedLIP::CoMAcc(real_t t) {
			BipedLIPKey* key = (BipedLIPKey*)traj.GetSegment(t).first;

			vec3_t a;
			if (key->next) {
				real_t l = 0.1;//���̒��S�������E�ܐ�܂ł̋���
				real_t tau_ssp = (key->period->val)*0.9;//���s����(SSP)
				real_t tau_dsp = (key->period->val)*0.1;//���s����(DSP)

				real_t T = param.T;
				real_t t0 = key->tick->time;
				real_t x0_ssp = key->com_pos_t[0]->val;
				real_t y0_ssp = key->com_pos_t[1]->val;
				real_t vx0_ssp = key->com_vel_t[0]->val;
				real_t vy0_ssp = key->com_vel_t[1]->val;
				real_t px = key->cop_pos_t[0]->val;
				real_t py = key->cop_pos_t[1]->val;

				real_t x0_dsp = px + l + (x0_ssp - px + l)*cosh(+tau_ssp / T) + T*(vx0_ssp);
				real_t y0_dsp = py + (y0_ssp - py)*cosh(tau_ssp / T) + T*vy0_ssp*sinh(tau_ssp / T);
				real_t vx0_dsp = 2 * l / tau_ssp + (x0_ssp - px + l) / T*sinh(tau_ssp / T) + (vx0_ssp - (2 * l / tau_ssp))*cosh(tau_ssp / T);
				real_t vy0_dsp = (y0_ssp - py) / T*sinh(tau_ssp / T) + vy0_ssp*cosh(tau_ssp / T);

				real_t dx = key->next->cop_pos_t[0]->val - key->cop_pos_t[0]->val -2*l;
				real_t dy = key->next->cop_pos_t[1]->val - key->cop_pos_t[1]->val;

				if (t0<t && t<(t0 + tau_ssp)) {
					a.x = (x0_ssp - px + l) / (T*T)*cosh((t - t0) / T) + (vx0_ssp - (2 * l / tau_ssp)) / T*sinh((t - t0) / T);
					a.y = (y0_ssp - py) / (T*T)*cosh((t - t0) / T) + vy0_ssp / T*sinh((t - t0) / T);
					a.z = 0.0;
				}
				else {
					a.x = (x0_dsp - px - l) / (T*T)*cosh((t - (t0 + tau_ssp)) / T) + (vx0_dsp - (dx / tau_dsp)) / T*sinh((t - (t0 + tau_ssp)) / T);
					a.y = (y0_dsp - py) / (T*T)*cosh((t - (t0 + tau_ssp)) / T) + (vy0_dsp - (dy / tau_dsp)) / T*sinh((t - (t0 + tau_ssp)) / T);
					a.z = 0.0;
				}
			}
			else {
				a.clear();
			}

			return a;
		}

		real_t BipedLIP::CoMOri(real_t t) {
			KeyPair      kp = traj.GetSegment(t);
			BipedLIPKey* cur = (BipedLIPKey*)kp.first;
			BipedLIPKey* next = (BipedLIPKey*)kp.second;

			real_t o;
			if (next) {
				real_t T = param.T;
				real_t t0 = cur->tick->time;
				real_t t1 = next->tick->time;
				real_t o0 = cur->com_pos_r->val;
				real_t o1 = next->com_pos_r->val;
				real_t r0 = cur->com_vel_r->val;
				real_t r1 = next->com_vel_r->val;

				o = InterpolatePos(t, t0, o0, r0, t1, o1, r1, Interpolate::Cubic);
			}
			else {
				o = cur->com_pos_r->val;
			}

			return o;
		}

		real_t BipedLIP::CoMAngVel(real_t t) {
			KeyPair      kp = traj.GetSegment(t);
			BipedLIPKey* cur = (BipedLIPKey*)kp.first;
			BipedLIPKey* next = (BipedLIPKey*)kp.second;

			real_t r;
			if (next) {
				real_t T = param.T;
				real_t t0 = cur->tick->time;
				real_t t1 = next->tick->time;
				real_t o0 = cur->com_pos_r->val;
				real_t o1 = next->com_pos_r->val;
				real_t r0 = cur->com_vel_r->val;
				real_t r1 = next->com_vel_r->val;

				r = InterpolateVel(t, t0, o0, r0, t1, o1, r1, Interpolate::Cubic);
			}
			else {
				r = cur->com_vel_r->val;
			}

			return r;
		}

		real_t BipedLIP::CoMAngAcc(real_t t) {
			KeyPair      kp = traj.GetSegment(t);
			BipedLIPKey* cur = (BipedLIPKey*)kp.first;
			BipedLIPKey* next = (BipedLIPKey*)kp.second;

			real_t a;
			if (next) {
				real_t T = param.T;
				real_t t0 = cur->tick->time;
				real_t t1 = next->tick->time;
				real_t o0 = cur->com_pos_r->val;
				real_t o1 = next->com_pos_r->val;
				real_t r0 = cur->com_vel_r->val;
				real_t r1 = next->com_vel_r->val;

				a = InterpolateAcc(t, t0, o0, r0, t1, o1, r1, Interpolate::Cubic);
			}
			else {
				a = 0.0;
			}

			return a;
		}

		vec3_t BipedLIP::SupFootPos(real_t t) {
			BipedLIPKey* key = (BipedLIPKey*)traj.GetSegment(t).first;

			return vec3_t(key->cop_pos_t[0]->val, key->cop_pos_t[1]->val, 0.0);
		}

		real_t BipedLIP::SupFootOri(real_t t) {
			BipedLIPKey* key = (BipedLIPKey*)traj.GetSegment(t).first;

			return key->cop_pos_r->val;
		}

		vec3_t BipedLIP::SwgFootPos(real_t t) {
			KeyPair      kp = traj.GetSegment(t);
			BipedLIPKey* cur = (BipedLIPKey*)kp.first;
			BipedLIPKey* next = (BipedLIPKey*)kp.second;
			BipedLIPKey* prev = (BipedLIPKey*)cur->prev;
			real_t       t0 = cur->tick->time;
			real_t       t1 = next->tick->time;
			real_t       h = t1 - t0;
			real_t       hhalf = h / 2.0;


			real_t		 ta = t0 + 0.05*h;//�����オ�肫�鎞��
			real_t		 tb = t0 + 0.95*h;//�����ڒn���鎞��

			real_t _2pi = 2.0 * M_PI;//2��
			real_t tau = (t - t0) / (ta - t0);//phase1
			real_t tau2 = (t - ta) / (tb - ta);//phase2
			real_t tau3 = (t1 - t) / (t1 - tb);//phase3

			real_t		 theta = (M_PI) / 12;//�ڒn�E���n���̊p�x
			real_t		 l = 0.1;//���̒��S�������E�ܐ�܂ł̋���
			real_t		 pa = l*(1 - cos(theta));//t0<t<ta��x�����̋���
			real_t		 pb = l*(1 - cos(theta));//tb<t<t1��x�����̋���


												 // �V�r�̎n�_�ƏI�_�͑O��̐ڒn�_
												 // 0���ڂ̎n�_��0���ڂ̎x�����̔��Α��ɐݒ�
												 // N-1���ڂ̏I�_��N-1���ڂ̎x�����̔��Α��ɐݒ�
			vec2_t p0, p1;
			// 1�`N-1����
			if (prev) {
				p0[0] = prev->cop_pos_t[0]->val;//�V�r�̎n�_�̈ʒux
				p0[1] = prev->cop_pos_t[1]->val;//				�@y
			}
			// 0����
			else {
				p0[0] = cur->swg_pos_t[0]->val;
				p0[1] = cur->swg_pos_t[1]->val;
			}

			if (next) {
				p1[0] = next->cop_pos_t[0]->val;//�V�r�̏I�_�̈ʒux
				p1[1] = next->cop_pos_t[1]->val;//				�@y
			}
			else {
				p1[0] = prev->cop_pos_t[0]->val;
				p1[1] = prev->cop_pos_t[1]->val;
			}

			real_t s;
			real_t z;
			if (h == 0.0) {//����
				s = 0.0;
				z = 0.0;
			}
			else {
				if (param.swingProfile == SwingProfile::Wedge) {
					if (t < t0 + hhalf) {
						z = param.swingHeight[0];
					}
					else {
						real_t a = (t - (t0 + hhalf)) / hhalf;
						z = (1 - a)*param.swingHeight[0] + a*param.swingHeight[1];
					}

					s = (t - t0) / h;//�����ɂ����鎞��t�̊���
				}

				if (param.swingProfile == SwingProfile::Cycloid) {//���E�ܐ�̗V�r�O��
					if (t < ta) {
						z = l*sin(theta*tau);
					}
					else if (ta <= t && t <= tb) {
						s = (tau2 - sin(_2pi*tau2) / _2pi);
						z = l*sin(theta) + (param.swingHeight[0] / 2.0) * (1 - cos(_2pi*tau2));
					}
					else {
						z = l*sin(theta*tau3);
					}

				}
			}

			vec3_t p;
			if (t < ta) {
				p[0] = p0[0] + l*(1 - cos(theta*tau));
				p[1] = p0[1];
			}
			else if (ta <= t && t <= tb) {
				p[0] = (1 - s) * (p0[0] + pa) + s * (p1[0] - pb);
				p[1] = (1 - s) * p0[1] + s * p1[1];
			}
			else {
				p[0] = p1[0] - l*(1 - cos(theta*tau3));
				p[1] = p1[1];
			}
			p[2] = z;

			return p;
		}

		real_t BipedLIP::SwgFootOri(real_t t) {
			KeyPair      kp = traj.GetSegment(t);
			BipedLIPKey* cur = (BipedLIPKey*)kp.first;
			BipedLIPKey* next = (BipedLIPKey*)kp.second;
			BipedLIPKey* prev = (BipedLIPKey*)cur->prev;
			real_t       t0 = cur->tick->time;
			real_t       t1 = next->tick->time;
			real_t       h = t1 - t0;

			real_t  q0, q1;
			if (prev)
				q0 = prev->cop_pos_r->val;
			else q0 = cur->swg_pos_r->val;

			if (next)
				q1 = next->cop_pos_r->val;
			else q1 = prev->cop_pos_r->val;

			real_t s;
			if (h == 0.0)
				s = 0.0;
			else s = (t - t0) / h;

			real_t q = (1 - s) * q0 + s * q1;

			return q;
		}


		vec3_t BipedLIP::TorsoPos(const vec3_t& pcom, const vec3_t& psup, const vec3_t& pswg) {
			// �R���p�X���f����蓷�̂̈ʒu�����߂�
			real_t a = param.torsoMassRatio;
			real_t b = param.legMassRatio;
			vec3_t p = (pcom - ((1 - a)*(1 - b) / 2)*(psup + pswg)) / (a + (1 - a)*b);
			return p;
		}

		//------------------------------------------------------------------------------------------------

		void BipedLIP::CalcTrajectory() {
			real_t tf = traj.back()->tick->time;
			real_t dt = 0.01;

			trajectory.clear();
			for (real_t t = 0.0; t < tf; t += dt) {
				TrajPoint tp;
				tp.t = t;
				tp.pos_com = CoMPos(t);
				tp.ori_com = CoMOri(t);
				tp.pos_sup = SupFootPos(t);
				tp.ori_sup = SupFootOri(t);
				tp.pos_swg = SwgFootPos(t);
				tp.ori_swg = SwgFootOri(t);
				tp.pos_torso = TorsoPos(tp.pos_com, tp.pos_sup, tp.pos_swg);

				trajectory.push_back(tp);
			}

			trajReady = true;
		}

		//------------------------------------------------------------------------------------------------

		void BipedLIP::Draw(DrawCanvas* canvas, DrawConfig* conf) {
			TrajectoryNode::Draw(canvas, conf);

			if (!trajReady)
				CalcTrajectory();

			if (trajectory.empty())
				return;

			// com
			if (conf->Set(canvas, DrawItem::BipedCoM, this)) {
				canvas->BeginLayer("biped_com", true);
				canvas->SetLineWidth(6.0f);
				canvas->BeginPath();
				canvas->MoveTo(trajectory[0].pos_com);
				for (uint i = 1; i < trajectory.size(); i++) {
					canvas->LineTo(trajectory[i].pos_com);
				}
				canvas->EndPath();
				canvas->EndLayer();
			}

			// torso
			if (conf->Set(canvas, DrawItem::BipedTorso, this)) {
				canvas->BeginLayer("biped_torso", true);
				canvas->SetLineWidth(3.0f);
				canvas->BeginPath();
				canvas->MoveTo(trajectory[0].pos_torso);
				for (uint i = 1; i < trajectory.size(); i++) {
					canvas->LineTo(trajectory[i].pos_torso);
				}
				canvas->EndPath();
				canvas->EndLayer();
			}


			// swing foot
			if (conf->Set(canvas, DrawItem::BipedSwing, this)) {
				canvas->BeginLayer("biped_swing", true);
				canvas->SetLineWidth(3.0f);
				canvas->BeginPath();
				canvas->MoveTo(trajectory[0].pos_swg);
				for (uint i = 1; i < trajectory.size(); i++) {
					if (trajectory[i - 1].pos_sup == trajectory[i].pos_sup) {
						canvas->LineTo(trajectory[i].pos_swg);
					}
					else {
						canvas->EndPath();
						canvas->BeginPath();
						canvas->MoveTo(trajectory[i].pos_swg);
					}
				}
				canvas->EndPath();
				canvas->EndLayer();
			}

			// double support snapshot
			/*		if (conf->Set(canvas, DrawItem::BipedDouble, this)) {
			canvas->BeginLayer("biped_double", true);
			canvas->SetLineWidth(1.0f);
			Vec3f p0, p1;
			for (uint i = 1; i < trajectory.size(); i++) {
			if (trajectory[i - 1].pos_sup != trajectory[i].pos_sup) {
			p0 = trajectory[i].pos_torso;
			p1 = trajectory[i].pos_sup;
			canvas->Line(p0, p1);

			p0 = trajectory[i].pos_torso;
			p1 = trajectory[i - 1].pos_sup;
			canvas->Line(p0, p1);
			}
			}
			canvas->EndLayer();
			}*/

		}

		void BipedLIP::DrawSnapshot(real_t time, DrawCanvas* canvas, DrawConfig* conf) {
			canvas->SetLineWidth(2.0f);
			canvas->BeginPath();
			canvas->MoveTo(CoMPos(time));
			canvas->LineTo(SupFootPos(time));
			canvas->MoveTo(CoMPos(time));
			canvas->LineTo(SwgFootPos(time));
			canvas->EndPath();
		}

		void BipedLIP::Save() {
			FILE* file1 = fopen("plan_step_case1(t_dt).csv", "w");
			fprintf(file1, "time, pos_com_x, pos_com_y, vel_com_x, vel_com_y,acc_com_x,acc_com_y\n");

			real_t dt = 0.01;
			real_t tf = traj.back()->tick->time;

			vec3_t pf = CoMPos(tf);
			vec3_t vf = CoMVel(tf);
			vec3_t af = CoMAcc(tf);

			for (real_t t = 0.0; t < tf; t += dt) {

				vec3_t p = CoMPos(t);
				vec3_t v = CoMVel(t);
				vec3_t a = CoMAcc(t);

				fprintf(file1, "%3.3lf, %3.3lf, %3.3lf,%3.3lf,%3.3lf,%3.3lf, %3.3lf\n", t, p.x, p.y, v.x, v.y, a.x, a.y);
			}
			fprintf(file1, "%3.3lf, %3.3lf, %3.3lf,%3.3lf,%3.3lf,%3.3lf, %3.3lf\n", tf, pf.x, pf.y, vf.x, vf.y, af.x, af.y);

			fclose(file1);




			FILE* file2 = fopen("plan_step_case1(t_k).csv", "w");
			fprintf(file2, "step, time, period, pos_com_x, pos_com_y, vel_com_x, vel_com_y, pos_cop_x, pos_cop_y\n");

			real_t t = 0.0;
			for (uint k = 0; k < graph->ticks.size(); k++) {
				BipedLIPKey* key = (BipedLIPKey*)traj.GetKeypoint(graph->ticks[k]);

				if (k != (graph->ticks.size()) - 1) {
					fprintf(file2, "%d, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf\n",
						k, t,
						key->period->val,
						key->com_pos_t[0]->val, key->com_pos_t[1]->val,
						key->com_vel_t[0]->val, key->com_vel_t[1]->val,
						key->cop_pos_t[0]->val, key->cop_pos_t[1]->val);
					t += key->period->val;
				}
				else {
					fprintf(file2, "%d, %3.3lf, 0, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf, %3.3lf\n",
						k, t,
						key->com_pos_t[0]->val, key->com_pos_t[1]->val,
						key->com_vel_t[0]->val, key->com_vel_t[1]->val,
						key->cop_pos_t[0]->val, key->cop_pos_t[1]->val);
				}
			}

			fclose(file2);
		}

		//-------------------------------------------------------------------------------------------------

		// Constructors(�R���X�g���N�^�˃N���X����I�u�W�F�N�g���쐬�����Ƃ��ɁC�����I�ɌĂяo�����)
		CoMCon::CoMCon(Solver* solver, int _tag, string _name, BipedLIPKey* _obj, uint _dir, real_t _scale) :
			Constraint(solver, 1, ID(_tag, _obj->node, _obj->tick, _name), _scale) {
			obj[0] = _obj;
			obj[1] = (BipedLIPKey*)_obj->next;
			dir = _dir;
		}

		CoMConTP::CoMConTP(Solver* solver, string _name, BipedLIPKey* _obj, uint _dir, real_t _scale) :
			CoMCon(solver, ConTag::BipedCoMTP, _name, _obj, _dir, _scale) {

			AddSLink(obj[0]->com_pos_t[dir]);
			AddSLink(obj[0]->com_vel_t[dir]);
			AddSLink(obj[0]->cop_pos_t[dir]);
			AddSLink(obj[0]->period);
			AddSLink(obj[1]->com_pos_t[dir]);
		}

		CoMConTV::CoMConTV(Solver* solver, string _name, BipedLIPKey* _obj, uint _dir, real_t _scale) :
			CoMCon(solver, ConTag::BipedCoMTV, _name, _obj, _dir, _scale) {

			AddSLink(obj[0]->com_pos_t[dir]);
			AddSLink(obj[0]->com_vel_t[dir]);
			AddSLink(obj[0]->cop_pos_t[dir]);
			AddSLink(obj[0]->period);
			AddSLink(obj[1]->com_vel_t[dir]);
		}

		CoMConR::CoMConR(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, real_t _scale) :
			Constraint(solver, 1, ID(ConTag::BipedCoMR, _obj->node, _obj->tick, _name), _scale) {
			obj[0] = _obj;
			obj[1] = (BipedLIPKey*)_obj->next;
			idx = _idx;

			AddSLink(obj[0]->com_pos_r);
			AddSLink(obj[0]->com_vel_r);
			AddSLink(obj[0]->period);
			AddSLink(obj[1]->com_pos_r);
			AddSLink(obj[1]->com_vel_r);
		}

		CoPConT::CoPConT(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, uint _dir, real_t _scale) :
			Constraint(solver, 1, ID(ConTag::BipedCoPT, _obj->node, _obj->tick, _name), _scale) {

			obj[0] = _obj;
			obj[1] = (BipedLIPKey*)_obj->next;
			idx = _idx;
			dir = _dir;

			AddSLink(obj[idx]->com_pos_t[0]);
			AddSLink(obj[idx]->com_pos_t[1]);
			AddSLink(obj[idx]->com_pos_r);
			AddSLink(obj[0]->cop_pos_t[0]);
			AddSLink(obj[0]->cop_pos_t[1]);
		}

		CoPConR::CoPConR(Solver* solver, string _name, BipedLIPKey* _obj, uint _idx, real_t _scale) :
			Constraint(solver, 1, ID(ConTag::BipedCoPR, _obj->node, _obj->tick, _name), _scale) {

			obj[0] = _obj;
			obj[1] = (BipedLIPKey*)_obj->next;
			idx = _idx;

			AddSLink(obj[idx]->com_pos_r);
			AddSLink(obj[0]->cop_pos_r);
		}

		//-------------------------------------------------------------------------------------------------

		void CoMCon::Prepare() {
			BipedLIP::Param& param = ((BipedLIP*)obj[0]->node)->param;
			l = 0.1;
			tau = obj[0]->period->val;
			T = param.T;
			t = obj[0]->period->val / T;
			ch = cosh(t);
			sh = sinh(t);
			p0 = obj[0]->com_pos_t[dir]->val;
			v0 = obj[0]->com_vel_t[dir]->val;
			c = obj[0]->cop_pos_t[dir]->val;
			p1 = obj[1]->com_pos_t[dir]->val;
			v1 = obj[1]->com_vel_t[dir]->val;
		}

		void CoMConTP::CalcCoef() {//�d�S�ʒu(����-�E��)��Δ���
			Prepare();
			if (dir == 0) {//x����
				((SLink*)links[0])->SetCoef(-ch);//xk-1
				((SLink*)links[1])->SetCoef(-T * sh);//vk-1
				((SLink*)links[2])->SetCoef(ch - 1.0);//pk
				((SLink*)links[3])->SetCoef(-((p0 - c + l) / T)*sh - 2 * l*T / (tau*tau)*sh - (v0 - (2 * l / tau))*ch);//tauk
				((SLink*)links[4])->SetCoef(1.0);//xk
			}
			else {//y����
				((SLink*)links[0])->SetCoef(-ch);
				((SLink*)links[1])->SetCoef(-T * sh);
				((SLink*)links[2])->SetCoef(ch - 1.0);
				((SLink*)links[3])->SetCoef(-((p0 - c) / T)*sh - v0*ch);
				((SLink*)links[4])->SetCoef(1.0);
			}
		}


		void CoMConTV::CalcCoef() {//�d�S���x(����-�E��)��Δ���
			Prepare();
			if (dir == 0) {//x����
				((SLink*)links[0])->SetCoef(-(1 / T)*sh);//xk-1
				((SLink*)links[1])->SetCoef(-ch);//vk-1
				((SLink*)links[2])->SetCoef((1 / T)*sh);//pk
				((SLink*)links[3])->SetCoef(2 * l / (tau*tau) - (p0 - c + l) / (T*T)*ch - 2 * l / (tau*tau)*ch - (v0 - (2 * l / tau)) / T*sh);//tauk
				((SLink*)links[4])->SetCoef(1.0);//vk
			}
			else {//y����
				((SLink*)links[0])->SetCoef(-(1 / T)*sh);
				((SLink*)links[1])->SetCoef(-ch);
				((SLink*)links[2])->SetCoef((1 / T)*sh);
				((SLink*)links[3])->SetCoef(-((p0 - c) / (T*T))*ch - (v0 / T)*sh);
				((SLink*)links[4])->SetCoef(1.0);
			}
		}


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



		void CoPConT::CalcCoef() {
			cx = obj[idx]->com_pos_t[0]->val;
			cy = obj[idx]->com_pos_t[1]->val;
			cr = obj[idx]->com_pos_r->val;
			px = obj[0]->cop_pos_t[0]->val;
			py = obj[0]->cop_pos_t[1]->val;
			_cos = cos(cr);
			_sin = sin(cr);

			if (dir == 0) {
				((SLink*)links[0])->SetCoef(-_cos);
				((SLink*)links[1])->SetCoef(-_sin);
				((SLink*)links[2])->SetCoef(-_sin*(px - cx) + _cos*(py - cy));
				((SLink*)links[3])->SetCoef(_cos);
				((SLink*)links[4])->SetCoef(_sin);
			}
			else {
				((SLink*)links[0])->SetCoef(_sin);
				((SLink*)links[1])->SetCoef(-_cos);
				((SLink*)links[2])->SetCoef(-_cos*(px - cx) - _sin*(py - cy));
				((SLink*)links[3])->SetCoef(-_sin);
				((SLink*)links[4])->SetCoef(_cos);
			}
		}



		void CoPConR::CalcCoef() {
			cr = obj[idx]->com_pos_r->val;
			pr = obj[0]->cop_pos_r->val;

			((SLink*)links[0])->SetCoef(-1.0);
			((SLink*)links[1])->SetCoef(1.0);
		}


		//-------------------------------------------------------------------------------------------------

		void CoMConTP::CalcDeviation() {//�d�S�ʒu(����-�E��)
			if (dir == 0) {//x����
				y[0] = p1 - 2 * l - (c - l) - (p0 - c + l)*ch - T*(v0 - (2 * l / tau))*sh;
			}
			else {//y����
				y[0] = p1 - c - (p0 - c)*ch - (v0*T)*sh;
			}
		}

		void CoMConTV::CalcDeviation() {//�d�S���x(����-�E��)
			if (dir == 0) {
				y[0] = v1 - 2 * l / tau - ((p0 - c + l) / T)*sh - (v0 - (2 * l / tau))*ch;
			}
			else {
				y[0] = v1 - ((p0 - c) / T)*sh - v0*ch;
			}
		}


		void CoMConR::CalcDeviation() {
			real_t s;
			if (idx == 0)
				s = 6.0*(q1 - q0) / tau2 - (4.0*w0 + 2.0*w1) / tau;
			else s = -6.0*(q1 - q0) / tau2 + (2.0*w0 + 4.0*w1) / tau;
			on_lower = (s < _min);
			on_upper = (s > _max);
			active = on_lower | on_upper;
			if (on_lower) y[0] = (s - _min);
			if (on_upper) y[0] = (s - _max);
		}



		void CoPConT::CalcDeviation() {
			real_t s;
			if (dir == 0)
				s = _cos*(px - cx) + _sin*(py - cy);
			else s = -_sin*(px - cx) + _cos*(py - cy);
			on_lower = (s < _min);
			on_upper = (s > _max);
			active = on_lower | on_upper;
			if (on_lower) y[0] = (s - _min);
			if (on_upper) y[0] = (s - _max);
		}



		void CoPConR::CalcDeviation() {
			real_t s = pr - cr;
			on_lower = (s < _min);
			on_upper = (s > _max);
			active = on_lower | on_upper;
			if (on_lower) y[0] = (s - _min);
			if (on_upper) y[0] = (s - _max);
		}


		//-------------------------------------------------------------------------------------------------


		void CoMConR::Project(real_t& l, uint k) {
			if (on_upper &&  l > 0.0) l = 0.0;
			if (on_lower &&  l < 0.0) l = 0.0;
			if (!on_upper && !on_lower) l = 0.0;
		}



		void CoPConT::Project(real_t& l, uint k) {
			if (on_upper &&  l > 0.0) l = 0.0;
			if (on_lower &&  l < 0.0) l = 0.0;
			if (!on_upper && !on_lower) l = 0.0;
		}



		void CoPConR::Project(real_t& l, uint k) {
			if (on_upper &&  l > 0.0) l = 0.0;
			if (on_lower &&  l < 0.0) l = 0.0;
			if (!on_upper && !on_lower) l = 0.0;
		}

	}
