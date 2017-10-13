#pragma once

#include <module/manager.h>

class Module;

struct ArgType{
	enum{
		Bool,
		Int,
		Real,
		String,
	};
};

class Argument : public UTRefCount{
public:
	string name;
	int	   type;
};

class Request : public UTRefCount{
public:
	string name;
	vector< UTRef<Argument> >	args;

	Request* AddArg(string _name, int _type);
};

struct ArgData{
	bool    boolean;
	int		integer;
	real_t  real;
	string	str;
};

/* ���N�G�X�g�}�l�W��
	- �����ŃN���e�B�J���Z�N�V�������g�p���X���b�h�Z�[�t�ɂȂ��Ă���

*/
class RequestManager : public Manager{
public:
	enum{
		HistoryMax = 100,
	};

	vector< UTRef<Request> >	requests;	///< �o�^���ꂽ���N�G�X�g
	
	deque<string>	history;	///< �������ꂽ���N�G�X�g�̗���
	
	deque<string>	queue;		///< ���N�G�X�g�L���[
	vector<string>	tokens;		///< ���N�G�X�g�𕪉������g�[�N����

	string name;				///< ���N�G�X�g��
	vector<ArgData>	args;		///< ���N�G�X�g����
	
public:
	bool     Parse (const vector<string>& tokens);
	
public:
	void     Write (XML& xml);
	Request* Add   (string _name);
	void	 Query (const string& line);
	bool     Handle();

	virtual void Read(XML& xml);
	
	virtual void Func();
	
	RequestManager();
};
