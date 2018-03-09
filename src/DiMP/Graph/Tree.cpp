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

	int nobj = 0;
	int njnt = 0;
	int ndof = 0;
	objects.clear();
	joints .clear();
	
	for(Object* obj : tree->objects){
		objects.push_back( (ObjectKey*)obj->traj.GetKeypoint(tick) );
		nobj++;
	}
	
	for(Joint* jnt : tree->joints){
		joints.push_back( (JointKey*)jnt->traj.GetKeypoint(tick) );
		njnt++;
		ndof += jnt->dof;
	}

	Jv.resize(nobj);
	Jw.resize(nobj);
	for(int i = 0; i < nobj; i++){
		Jv[i].resize(ndof);
		Jw[i].resize(ndof);
	}

}

void TreeKey::AddCon(Solver* solver){

}

void TreeKey::Prepare(){
	Tree* tree = (Tree*)node;

	// ƒ„ƒRƒrƒAƒ“
	int i = 0, j, j2;
	for(ObjectKey* obj : objects){
		j  = 0;
		j2 = 0;
		for(JointKey* jnt : joints){
			int ndof = ((Joint*)jnt->node)->dof;
			for(int n = 0; n < ndof; n++){
				if(tree->dependent[i][j]){
					Jv[i][j2] = jnt->Jv[n] + jnt->Jw[n] % (obj->pos_t->val - (jnt->sockObj->pos_t->val + jnt->r[n]));
					Jw[i][j2] = jnt->Jw[n];
				}
				else{
					Jv[i][j2].clear();
					Jw[i][j2].clear();
				}
				j2++;
			}
			j++;
		}
		i++;
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
	parObject.clear();
	parJoint .clear();
	dependent.clear();
	int dofTotal = 0;

	root->tree      = this;
	root->treeIndex = 0;
	objects  .push_back(root);
	parObject.push_back(-1);
	parJoint .push_back(-1);

	bool updated;
	do{
		updated = false;
		for(Joint* jnt : graph->joints){
			if(jnt->tree)
				continue;

			Object* sockObj = jnt->sock->obj;
			Object* plugObj = jnt->plug->obj;

			if(sockObj->tree == this && !plugObj->tree){
				plugObj->tree         = this;
				plugObj->treeIndex    = (int)objects.size();
				jnt    ->tree         = this;
				jnt    ->treeIndex    = (int)joints.size();
				jnt    ->treeDofIndex = dofTotal;
				objects.push_back(plugObj);
				joints .push_back(jnt    );
				dofTotal += jnt->dof;

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

	// ˆË‘¶ŠÖŒW
	uint nobj = (uint)objects.size();
	uint njnt = (uint)joints .size();
	dependent.resize(nobj);
	for(uint i = 0; i < nobj; i++){
		dependent[i].resize(njnt);
		for(uint j = 0; j < njnt; j++)
			dependent[i][j] = IsDependent(i, j);
	}

}

void Tree::Prepare(){
	TrajectoryNode::Prepare();

}

//-------------------------------------------------------------------------------------------------

void Trees::Extract(){
	for(int i = 0; i < (int)size(); i++)
		at(i)->Extract();
}

void Trees::ForwardKinematics(){
	for(int i = 0; i < (int)size(); i++)
		at(i)->root->ForwardKinematics();
}

}
