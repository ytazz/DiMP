#pragma once

#include <DiMP2/Types.h>

#include <sbxml.h>

namespace DiMP2{;

class Graph;

class Loader{
public:
	bool Load(XML& xml     , Graph* graph);
	bool Load(XMLNode* node, Graph* graph);

	void LoadTicks       (XMLNode* node, Graph* graph);
	void LoadGeometry    (XMLNode* node, Graph* graph);
	void LoadObject      (XMLNode* node, Graph* graph);
	void LoadConnector   (XMLNode* node, Graph* graph);
	void LoadJoint       (XMLNode* node, Graph* graph);
	void LoadTimeSlot    (XMLNode* node, Graph* graph);
	void LoadTask        (XMLNode* node, Graph* graph);
	void LoadScaling     (XMLNode* node, Graph* graph);
	void LoadEnable      (XMLNode* node, Graph* graph);
	void LoadPriority    (XMLNode* node, Graph* graph);
	void LoadIteration   (XMLNode* node, Graph* graph);
	void LoadCorrection  (XMLNode* node, Graph* graph);

	Loader();
};

}
