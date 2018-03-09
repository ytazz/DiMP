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

}
