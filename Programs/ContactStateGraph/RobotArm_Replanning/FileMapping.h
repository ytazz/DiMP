#ifndef __FILEMAPPING_H
#define __FILEMAPPING_H

//
// 共有メモリを使ってプロセス間通信をしたいときに使うクラス
// 
// 2014-10-15追記
// タスクスケジューリング用途でDiMP/Programs/Robotarmに持ってきた。
// 
// 2012-11-04	石井雅人
//



#include <iostream>
#include <string>
#define NOMINMAX
#include <Windows.h>
#include <tchar.h>


using namespace std;


template< class T >
class FileMapping
{
private:
	HANDLE FMapFile;	// FileMappingオブジェクト
	HANDLE FMutex;	// Mutexオブジェクト
	T *FMapAddress;	// 共有メモリ上にあるTを指すポインタ

public:
	FileMapping( LPCTSTR fm_name = L"TS_TP_SHMEM", LPCTSTR m_name = L"TS_TP_MUTEX" );
	~FileMapping();

	void readTo( T *t );	// 共有メモリのデータをtに入れる
	void write( const T *t );	// tを共有メモリに書き込む
	T* get();	// Tを指すポインタを返す
};


// fm_name	FileMappingオブジェクトの名前
// m_name	Mutexオブジェクトの名前

template< class T >	
FileMapping<T>::FileMapping( LPCTSTR fm_name, LPCTSTR m_name )
{
	FMapFile = NULL;
	FMutex = NULL;
	FMapAddress = NULL;

//	cout << "<!-- FileMapping コンストラクタ -->" << '\n';
//	cout << "sizeof(" << typeid(T).name() << ") : " << sizeof(T) << '\n';

	FMapFile = OpenFileMapping(
					FILE_MAP_ALL_ACCESS,
					FALSE,
					fm_name
				);
	if( FMapFile != NULL ) {
//		cout << "OpenFileMapping ( " << fm_name << " ) 成功 : " << FMapFile << '\n';
	} else {
		FMapFile = CreateFileMapping(
						INVALID_HANDLE_VALUE,
						NULL,
						PAGE_READWRITE,
						0,
						sizeof(T),
						fm_name
					);
		if( FMapFile == NULL ) { cerr<<"cfm error\n"; throw; }
//		cout << "CreateFileMapping ( " << fm_name << " ) 成功 : " << FMapFile << '\n';
	}

	FMutex = CreateMutex(
				NULL,
				FALSE,
				m_name
			);
	if( FMutex == NULL ) { cerr<<"cm error\n"; throw; }
//	cout << "CreateMutex 成功 " << m_name << " : " << FMutex << '\n';

//	cout << "<!-- コンストラクタ完了 -->" << '\n';
}

template< class T >
FileMapping<T>::~FileMapping()
{
//	cout << "FMapFile :\t" << FMapFile << '\n';
//	cout << "FMutex :\t" << FMutex << '\n';
//	cout << "FMapAddress :\t" << FMapAddress << '\n';

	UnmapViewOfFile( FMapAddress );
	CloseHandle( FMapFile );
	CloseHandle( FMutex );

//	cout << "デストラクタ完了" << '\n';
}

template< class T >
void FileMapping<T>::readTo( T *t ) {
	// ファイルマッピングオブジェクトが閉じられていたら
	if( !FMapFile ) { 
		cerr << '\a' << "!!! error !!! " << "ファイルマッピングオブジェクトが閉じられています。" << '\n';
		return;
	}

	// アドレス空間にファイルのビューをマップする
	FMapAddress = (T *)MapViewOfFile( FMapFile, FILE_MAP_READ, 0, 0, sizeof(T) );
	if( FMapAddress == NULL ) {
		cerr << '\a' << "!!! error !!! " << "マッピングに失敗しました。" << '\n';
		return;
	} else {
//		cout << "マッピングに成功しました。" << '\n';
	
	
	// ミューテックスオブジェクトの状態をチェックする。
	WaitForSingleObject( FMutex, INFINITE );

	// データをコピーする
	memcpy( t, FMapAddress, sizeof(T) );
	
	// ミューテックスオブジェクトを解放する
	ReleaseMutex( FMutex );

//	cout << "読み込み完了" << '\n';
}
}

template< class T >
void FileMapping<T>::write( const T *t ) {
	// ファイルマッピングオブジェクトが閉じられていたら
	if( !FMapFile ) { 
		cerr << '\a' << "!!! error !!! " << "ファイルマッピングオブジェクトが閉じられています。" << '\n';
		return;
	}

	// アドレス空間にファイルのビューをマップする
	FMapAddress = (T *)MapViewOfFile( FMapFile, FILE_MAP_WRITE, 0, 0, sizeof(T) );
	if( FMapAddress == NULL ) {
		cerr << '\a' << "!!! error !!! " << "マッピングに失敗しました。" << '\n';
		return;
	} else {
//		cout << "マッピングに成功しました" << '\n';
	}

	// ミューテックスオブジェクトの状態をチェックする。
	WaitForSingleObject( FMutex, INFINITE );

	// データを書き込む
	memcpy( FMapAddress, t, sizeof(T) );
	// フラッシュして確実に更新する
	FlushViewOfFile( FMapAddress, sizeof(T) );
//	cout << "フラッシュしました。" << '\n';

	ReleaseMutex( FMutex );

//	cout << "書き込み完了" << '\n';
}

template< class T >
T* FileMapping<T>::get()
{
	if(!hFile) { return (T*)0; }
	else {	return (T*)MapViewOfFile(hFile, FILE_MAP_WRITE, 0, 0, sizeof(T)); }
}


#endif