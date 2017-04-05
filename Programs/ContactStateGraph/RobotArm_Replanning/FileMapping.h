#ifndef __FILEMAPPING_H
#define __FILEMAPPING_H

//
// ���L���������g���ăv���Z�X�ԒʐM���������Ƃ��Ɏg���N���X
// 
// 2014-10-15�ǋL
// �^�X�N�X�P�W���[�����O�p�r��DiMP/Programs/Robotarm�Ɏ����Ă����B
// 
// 2012-11-04	�Έ��l
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
	HANDLE FMapFile;	// FileMapping�I�u�W�F�N�g
	HANDLE FMutex;	// Mutex�I�u�W�F�N�g
	T *FMapAddress;	// ���L��������ɂ���T���w���|�C���^

public:
	FileMapping( LPCTSTR fm_name = L"TS_TP_SHMEM", LPCTSTR m_name = L"TS_TP_MUTEX" );
	~FileMapping();

	void readTo( T *t );	// ���L�������̃f�[�^��t�ɓ����
	void write( const T *t );	// t�����L�������ɏ�������
	T* get();	// T���w���|�C���^��Ԃ�
};


// fm_name	FileMapping�I�u�W�F�N�g�̖��O
// m_name	Mutex�I�u�W�F�N�g�̖��O

template< class T >	
FileMapping<T>::FileMapping( LPCTSTR fm_name, LPCTSTR m_name )
{
	FMapFile = NULL;
	FMutex = NULL;
	FMapAddress = NULL;

//	cout << "<!-- FileMapping �R���X�g���N�^ -->" << '\n';
//	cout << "sizeof(" << typeid(T).name() << ") : " << sizeof(T) << '\n';

	FMapFile = OpenFileMapping(
					FILE_MAP_ALL_ACCESS,
					FALSE,
					fm_name
				);
	if( FMapFile != NULL ) {
//		cout << "OpenFileMapping ( " << fm_name << " ) ���� : " << FMapFile << '\n';
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
//		cout << "CreateFileMapping ( " << fm_name << " ) ���� : " << FMapFile << '\n';
	}

	FMutex = CreateMutex(
				NULL,
				FALSE,
				m_name
			);
	if( FMutex == NULL ) { cerr<<"cm error\n"; throw; }
//	cout << "CreateMutex ���� " << m_name << " : " << FMutex << '\n';

//	cout << "<!-- �R���X�g���N�^���� -->" << '\n';
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

//	cout << "�f�X�g���N�^����" << '\n';
}

template< class T >
void FileMapping<T>::readTo( T *t ) {
	// �t�@�C���}�b�s���O�I�u�W�F�N�g�������Ă�����
	if( !FMapFile ) { 
		cerr << '\a' << "!!! error !!! " << "�t�@�C���}�b�s���O�I�u�W�F�N�g�������Ă��܂��B" << '\n';
		return;
	}

	// �A�h���X��ԂɃt�@�C���̃r���[���}�b�v����
	FMapAddress = (T *)MapViewOfFile( FMapFile, FILE_MAP_READ, 0, 0, sizeof(T) );
	if( FMapAddress == NULL ) {
		cerr << '\a' << "!!! error !!! " << "�}�b�s���O�Ɏ��s���܂����B" << '\n';
		return;
	} else {
//		cout << "�}�b�s���O�ɐ������܂����B" << '\n';
	
	
	// �~���[�e�b�N�X�I�u�W�F�N�g�̏�Ԃ��`�F�b�N����B
	WaitForSingleObject( FMutex, INFINITE );

	// �f�[�^���R�s�[����
	memcpy( t, FMapAddress, sizeof(T) );
	
	// �~���[�e�b�N�X�I�u�W�F�N�g���������
	ReleaseMutex( FMutex );

//	cout << "�ǂݍ��݊���" << '\n';
}
}

template< class T >
void FileMapping<T>::write( const T *t ) {
	// �t�@�C���}�b�s���O�I�u�W�F�N�g�������Ă�����
	if( !FMapFile ) { 
		cerr << '\a' << "!!! error !!! " << "�t�@�C���}�b�s���O�I�u�W�F�N�g�������Ă��܂��B" << '\n';
		return;
	}

	// �A�h���X��ԂɃt�@�C���̃r���[���}�b�v����
	FMapAddress = (T *)MapViewOfFile( FMapFile, FILE_MAP_WRITE, 0, 0, sizeof(T) );
	if( FMapAddress == NULL ) {
		cerr << '\a' << "!!! error !!! " << "�}�b�s���O�Ɏ��s���܂����B" << '\n';
		return;
	} else {
//		cout << "�}�b�s���O�ɐ������܂���" << '\n';
	}

	// �~���[�e�b�N�X�I�u�W�F�N�g�̏�Ԃ��`�F�b�N����B
	WaitForSingleObject( FMutex, INFINITE );

	// �f�[�^����������
	memcpy( FMapAddress, t, sizeof(T) );
	// �t���b�V�����Ċm���ɍX�V����
	FlushViewOfFile( FMapAddress, sizeof(T) );
//	cout << "�t���b�V�����܂����B" << '\n';

	ReleaseMutex( FMutex );

//	cout << "�������݊���" << '\n';
}

template< class T >
T* FileMapping<T>::get()
{
	if(!hFile) { return (T*)0; }
	else {	return (T*)MapViewOfFile(hFile, FILE_MAP_WRITE, 0, 0, sizeof(T)); }
}


#endif