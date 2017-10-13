#include <module/request.h>
#include <module/rendering.h>
#include <module/module.h>

#include <sbtokenizer.h>
#include <sbconsole.h>
using Scenebuilder::Tokenizer;
using Scenebuilder::string_iterator_pair;

#include <windows.h>

///////////////////////////////////////////////////////////////////////////////////////////////////

Request* Request::AddArg(string _name, int _type){
	Argument* a = new Argument();
	a->name = _name;
	a->type = _type;
	args.push_back(a);
	return this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

RequestManager::RequestManager(){
	// 終了
	Add("q"       );
	Add("quit"    );
}

void RequestManager::Read(XML& xml){
	// リクエスト履歴を読み込む
	string contents = xml.GetRootNode()->GetContents();

	// 改行文字で分割
	Tokenizer tok;
	for(tok.Set(contents, "\n", true); !tok.IsEnd(); tok.Next()){
		string_iterator_pair str = tok.GetToken();
		if(!str.empty())
			history.push_back(to_string(str));
	}
}

void RequestManager::Write(XML& xml){
	stringstream ss;
	for(deque<string>::iterator it = history.begin(); it != history.end(); it++){
		if(it != history.begin())
			ss << endl;
		ss << *it;
	}
	xml.CreateNode("history", -1);
	xml.GetRootNode()->SetContents(ss.str());
}

Request* RequestManager::Add(string _name){
	Request* r = new Request();
	r->name = _name;
	requests.push_back(r);
	return r;
}

bool RequestManager::Parse(const vector<string>& tokens){
	name = tokens[0];
	args.clear();

	// 名前が一致するリクエストを集める
	vector<Request*> reqs;
	for(uint i = 0; i < requests.size(); i++){
		Request* r = requests[i];
		if(r->name == name)
			reqs.push_back(r);
	}
	if(reqs.empty()){
		Message::Error("request not recognized: %s", name.c_str());
		return false;
	}

	// その中から引数の数が一致するものを探す
	Request* req = 0;
	for(uint i = 0; i < reqs.size(); i++){
		Request* r = reqs[i];
		if(r->args.size() == tokens.size()-1){
			req = r;
			break;
		}
	}
	if(!req){
		Message::Error("invalid argument:");
		for(uint i = 0; i < reqs.size(); i++){
			Request* r = reqs[i];

			string str;
			str += name;
			for(uint j = 0; j < r->args.size(); j++){
				str += " ";
				str += r->args[j]->name;
			}
			Message::Out("%s", str.c_str());
		}
		return false;
	}

	bool ok = true;
	for(uint i = 0; i < req->args.size(); i++){
		ArgData dat;
		const string& tok = tokens[i+1];
		if(req->args[i]->type == ArgType::Bool){
			try{
				Converter::FromString(tok, dat.boolean);
			}
			catch(Exception&){
				Message::Out("%s: argument %s must be boolean", name.c_str(), req->args[i]->name.c_str());
				ok = false;
			}
		}
		if(req->args[i]->type == ArgType::Int){
			try{
				Converter::FromString(tok, dat.integer);
			}
			catch(Exception&){
				Message::Out("%s: argument %s must be integer", name.c_str(), req->args[i]->name.c_str());
				ok = false;
			}
		}
		if(req->args[i]->type == ArgType::Real){
			try{
				Converter::FromString(tok, dat.real);
			}
			catch(Exception&){
				Message::Out("%s: argument %s must be real", name.c_str(), req->args[i]->name.c_str());
				ok = false;
			}
		}
		if(req->args[i]->type == ArgType::String){
			dat.str = tok;
		}
		args.push_back(dat);
	}
	return ok;
}

void RequestManager::Query(const string& line){
	// ';'はコマンド区切り
	Tokenizer tokenizer(line, ";", true);

	cs.Enter();
	while(!tokenizer.IsEnd()){
		queue.push_back(to_string(eat_white(tokenizer.GetToken())));
		tokenizer.Next();
	}
	cs.Leave();
}

bool RequestManager::Handle(){
	Module* mod   = Module::Get();
	
	if(queue.empty())
		return false;
	
	cs.Enter();
	string line = queue.front();
	queue.pop_front();
	cs.Leave();
	
	// 空文と終了コマンド以外は履歴へ追加
	if(!(line == "q" || line == "quit" || line.empty())){
		history.push_back(line);
		if(history.size() > HistoryMax)
			history.pop_front();
	}

	// コマンド文を分解
	tokens.clear();
	Tokenizer::Split(tokens, line, " \t", true);
	if(tokens.empty())
		return false;
	// 解析
	if(!Parse(tokens))
		return false;

	return true;
}

void RequestManager::Func(){
	static bool prompt = true;
	Module* mod = Module::Get();
	
	while(!mod->evExit.IsSet()){
		if(prompt){
			Scenebuilder::Console::SetCursorPosition(0, mod->renManager->consoleInputRow);
			Scenebuilder::Console::Fill   (0, mod->renManager->consoleInputRow, mod->renManager->consoleWidth, 1, ' ');
			Scenebuilder::Console::Refresh(mod->renManager->consoleInputRow, 1);
			
			printf("input command: ");
			prompt = false;
		}

		// 入力受付中にevExitが立つかも知れないので100msでタイムアウトする
		fflush(stdin);
		HANDLE hstdin = GetStdHandle(STD_INPUT_HANDLE);
		int ret = WaitForSingleObject(hstdin, 100);
		if(ret == WAIT_OBJECT_0){
			string line;
			getline(cin, line);

			if(!line.empty())
				Query(line);

			prompt = true;
		}		
	}
}
