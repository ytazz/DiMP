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

/* リクエストマネジャ
	- 内部でクリティカルセクションを使用しスレッドセーフになっている

*/
class RequestManager : public Manager{
public:
	enum{
		HistoryMax = 100,
	};

	vector< UTRef<Request> >	requests;	///< 登録されたリクエスト
	
	deque<string>	history;	///< 処理されたリクエストの履歴
	
	deque<string>	queue;		///< リクエストキュー
	vector<string>	tokens;		///< リクエストを分解したトークン列

	string name;				///< リクエスト名
	vector<ArgData>	args;		///< リクエスト引数
	
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
