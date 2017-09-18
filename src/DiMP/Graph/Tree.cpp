#include <DiMP/Graph/Tree.h>
#include <DiMP/Graph/Object.h>
#include <DiMP/Graph/Joint.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Render/Config.h>

namespace DiMP{;

//-------------------------------------------------------------------------------------------------
// TreeKey

void TreeKey::AddVar(Solver* solver){
	Tree* tree = (Tree*)node;

	uint nobj = (uint)tree->objects  .size();
	uint njnt = (uint)tree->joints   .size();
	uint nadj = (uint)tree->adjacents.size();

	objects.resize(nobj);
	for(uint i = 0; i < nobj; i++)
		objects[i] = (ObjectKey*)tree->objects[i]->traj.GetKeypoint(tick);
	
	joints.resize(njnt);
	for(uint i = 0; i < njnt; i++)
		joints[i] = (JointKey*)tree->joints[i]->traj.GetKeypoint(tick);

	adjacents.resize(nadj);
	for(uint i = 0; i < nadj; i++){
		adjacents[i].joint = (JointKey*)tree->adjacents[i].joint->traj.GetKeypoint(tick);
		adjacents[i].sock  = tree->adjacents[i].sock;
		adjacents[i].iobj  = tree->adjacents[i].iobj;
	}

	Jv.resize(nobj);
	Jw.resize(nobj);
	for(uint i = 0; i < nobj; i++){
		Jv[i].resize(njnt);
		Jw[i].resize(njnt);
	}

	M.resize(njnt);
	for(uint i = 0; i < njnt; i++)
		M[i].resize(njnt);

}

void TreeKey::AddCon(Solver* solver){
	if(next){
		TreeKey* nextTree = (TreeKey*)next;

		// equation of motion constraint
		con_eom.resize(joints.size());
		for(uint i = 0; i < joints.size(); i++){
			stringstream ss;
			ss << i;
			real_t sc = ((Joint*)joints[i]->node)->IsRotational(0) ? node->graph->scale.force_r : node->graph->scale.force_t;
			con_eom[i] = new EomCon(solver, name + "_eom" + ss.str(), this, i, sc);
		}

	}
}

int TreeKey::GetIndex(ObjectKey* obj){
	for(uint i = 0; i < objects.size(); i++){
		if(objects[i] == obj)
			return i;
	}
	return -1;
}

void TreeKey::Prepare(){
	Tree* tree = (Tree*)node;

	uint nobj = (uint)objects.size();
	uint njnt = (uint)joints .size();

	// ヤコビアン
	for(uint i = 0; i < nobj; i++){
		ObjectKey* obj = objects[i];

		for(uint j = 0; j < njnt; j++){
			JointKey* jnt = joints[j];

			if(tree->dependent[i][j]){
				Jv[i][j] = jnt->Jv[0] + jnt->Jw[0] % (obj->pos_t->val - (jnt->sockObj->pos_t->val + jnt->r[0]));
				Jw[i][j] = jnt->Jw[0];
			}
			else{
				Jv[i][j].clear();
				Jw[i][j].clear();
			}
		}
	}

	// 慣性行列
	for(uint j0 = 0; j0 < njnt; j0++){
		for(uint j1 = 0; j1 < njnt; j1++){
			M[j0][j1] = 0.0;

			for(uint i = 0; i < nobj; i++){
				Object* obj = (Object*)objects[i]->node;
				real_t m = obj->param.mass;
				real_t I = obj->param.inertia;

				if(tree->dependent[i][j0] && tree->dependent[i][j1]){
					M[j0][j1] += m * Jv[i][j0] * Jv[i][j1] + I * Jw[i][j0] * Jw[i][j1];
				}
			}
		}
	}

}

//-------------------------------------------------------------------------------------------------
// Tree

Tree::Tree(Graph* g, string n):TrajectoryNode(g, n){
	root = 0;
	type = Type::Tree;
	graph->trees.Add(this);
}

Tree::~Tree(){
	graph->trees.Remove(this);
}

bool Tree::IsDependent(int i, int j){
	if(parObject[i] == -1)
		return false;
	if(parJoint[i] == j)
		return true;
	return IsDependent(parObject[i], j);
}

void Tree::Extract(){
	objects  .clear();
	joints   .clear();
	adjacents.clear();
	parObject.clear();
	parJoint .clear();
	dependent.clear();

	root->tree = this;
	objects  .push_back(root);
	parObject.push_back(-1);
	parJoint .push_back(-1);

	bool updated;
	do{
		updated = false;
		for(uint i = 0; i < graph->joints.size(); i++){
			Joint* jnt = graph->joints[i];
			if(jnt->tree)
				continue;

			Object* sockObj = jnt->sock->obj;
			Object* plugObj = jnt->plug->obj;

			if(sockObj->tree == this && !plugObj->tree){
				plugObj->tree = this;
				jnt    ->tree = this;
				objects.push_back(plugObj);
				joints .push_back(jnt    );

				uint isock;
				for(isock = 0; isock < objects.size(); isock++){
					if(objects[isock] == sockObj)
						break;
				}
				parObject.push_back(isock);
				parJoint .push_back((int)joints.size() - 1);
				updated = true;
			}
		}
	}
	while(updated);

	// 依存関係
	uint nobj = (uint)objects.size();
	uint njnt = (uint)joints .size();
	dependent.resize(nobj);
	for(uint i = 0; i < nobj; i++){
		dependent[i].resize(njnt);
		for(uint j = 0; j < njnt; j++)
			dependent[i][j] = IsDependent(i, j);
	}

	// ツリーに属さないが隣接している関節を抽出
	for(uint i = 0; i < nobj; i++){
		Object* obj = objects[i];

		for(uint c = 0; c < obj->cons.size(); c++){
			Connector* con = obj->cons[c];

			for(uint j0 = 0; j0 < con->joints.size(); j0++){
				Joint* jnt = con->joints[j0];

				bool found = false;
				for(uint j1 = 0; j1 < joints.size(); j1++){
					if(joints[j1] == jnt){
						found = true;
						break;
					}
				}
				if(!found)
					adjacents.push_back(Adjacent(jnt, jnt->sock == con, i));
			}
		}
	}

	//DSTR << "objects  : " << objects  .size() << endl;
	//DSTR << "joints   : " << joints   .size() << endl;
	//DSTR << "adjacents: " << adjacents.size() << endl;

}

void Tree::Prepare(){
	TrajectoryNode::Prepare();

}

//-------------------------------------------------------------------------------------------------

void Trees::Extract(){
	for(uint i = 0; i < size(); i++)
		at(i)->Extract();
}

void Trees::ForwardKinematics(){
	for(uint i = 0; i < size(); i++)
		at(i)->root->ForwardKinematics();
}

//-------------------------------------------------------------------------------------------------

EomCon::EomCon(Solver* solver, string _name, TreeKey* _tree, uint _idx, real_t _scale):
	Constraint(solver, 1, ID(ConTag::Eom, _tree->node, _tree->tick, _name), _scale){
	tree[0] = _tree;
	tree[1] = (TreeKey*)_tree->next;
	idx     = _idx;
	
	uint njnt = (uint)tree[0]->joints   .size();
	uint nadj = (uint)tree[0]->adjacents.size();

	// joint vel
	for(uint i = 0; i < njnt; i++)
		AddSLink(tree[0]->joints[i]->vel[0]);
	for(uint i = 0; i < njnt; i++)
		AddSLink(tree[1]->joints[i]->vel[0]);

	// joint torque
	AddSLink(tree[0]->joints[idx]->torque[0]);
	
	/*
	// external force
	for(uint i = 0; i < nadj; i++){
		AddRLink(tree[0]->adjacents[i].joint->force_t);
		AddRLink(tree[0]->adjacents[i].joint->force_r);
	}
	*/
}

void EomCon::CalcCoef(){
	uint njnt = (uint)tree[0]->joints   .size();
	uint nadj = (uint)tree[0]->adjacents.size();

	real_t h   = tree[0]->hnext;

	int l = 0;

	// joint vel
	for(uint i = 0; i < njnt; i++)
		((SLink*)links[l++])->SetCoef(-tree[0]->M[idx][i] / h);
	for(uint i = 0; i < njnt; i++)
		((SLink*)links[l++])->SetCoef( tree[0]->M[idx][i] / h);

	// joint torque
	((SLink*)links[l++])->SetCoef(-1.0);

	/*
	// external force
	for(uint i = 0; i < nadj; i++){
		TreeKey::Adjacent& adj = tree[0]->adjacents[i];
		JointKey* jnt = adj.joint;
		vec3_t Jv = tree[0]->Jv[adj.iobj][idx];
		vec3_t Jw = tree[0]->Jw[adj.iobj][idx];
		vec3_t r  = (adj.sock ? jnt->r[0] + jnt->q[0] * jnt->rrel : jnt->r[1]);
		((RLink*)links[l++])->SetCoef((adj.sock ? 1.0 : -1.0) * (Jv + Jw % r));
		((RLink*)links[l++])->SetCoef((adj.sock ? 1.0 : -1.0) * Jw);
	}
	*/
}

void EomCon::CalcDeviation(){
	Constraint::CalcDeviation();
	
	uint nobj = (uint)tree[0]->objects.size();

	/*
	// 外力による項
	for(uint i = 0; i < nobj; i++){
		ObjectKey* obj = tree[0]->objects[i];
		if(!obj->GetNode()->param.dynamical)
			continue;

		y[0] -= (tree[0]->Jv[i][idx] * obj->fext_t + tree[0]->Jw[i][idx] * obj->fext_r);
	}
	*/
}

}
