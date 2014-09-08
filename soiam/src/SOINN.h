/////////////////////////////
// SOINN Version 3.02      //
// Managed by Daiki Kimura //
/////////////////////////////

#ifndef _SELF_ORGANIZING_INCREMENTAL_NEURAL_NETWORK_HEADER_
#define _SELF_ORGANIZING_INCREMENTAL_NEURAL_NETWORK_HEADER_

#include <vector>
#include <string>
#include <cstring>
#include <sys/types.h>

using namespace std;

// If change this infoData, you should change Load and Output func.
typedef struct {
	int fileName;
	int cannyTh1;
	int cannyTh2;
	int x;
	int y;
	int width;
	int height;
	string firstImageClass;
} soinnNodeInfoData;

class CEdge
{
	friend class CSOINN;
	friend class COutputWindow;

	// Constant
public:
	static const int EMPTY/*=-1*/;

	// Function
public:
	CEdge();
	CEdge(const int node1, const int node2);
	~CEdge();

private:
	void	Init(void);
	void	Copy(const CEdge &edge);
	bool	Replace(const int before, const int after);

	// Variable
private:
	int		m_from;
	int		m_to;
	int		m_age;
};

class CNode
{
	friend class CSOINN;

	// Constant
public:
	static const int	MAX_NEIGHBOR/*=30*/;
	static const int	EMPTY/*=-1*/;

	// Function
public:
	CNode(int);
	CNode(const double *signal,int);
	CNode(const double *signal,int,soinnNodeInfoData);
	~CNode();
	bool	AddNeighbor(const int node);
	bool	DeleteNeighbor(const int node);
	bool	ReplaceNeighbor(const int before, const int after);
	bool	IsNeighbor(const int node);
	double* GetSignal();
	int     GetClass();
	bool	GetIsNodeRemoved();
	void	SetIsNodeRemoved(const bool isNodeRemoved);
	bool	GetIsEdgeRemoved();
	void	SetIsEdgeRemoved(const bool isEdgeRemoved);

	int		GetNeighborNum();
	int		GetLearningTime();

	void AddInfo(soinnNodeInfoData);

private:
	void	Init(int);
	void	Copy(const CNode &src);

	// Variable
private:
public:
	double*	m_signal;
	vector<int> m_neighbor;
	int		m_neighborNum;
	int		m_learningTime;
	int		m_classID;
	int	    m_dimension;
	bool	m_isNodeRemoved;
	bool	m_isEdgeRemoved;
	vector<soinnNodeInfoData> infos;
};

class CSOINN
{
public:
	static const int    NO_CHANGE/*=0*/;
	static const int	UNCLASSIFIED/*=-1*/;
	static const int	NOT_FOUND/*=-1*/;
	static const double	INFI/*=1e10*/;

public:
	CSOINN(const int dimension, const int removeNodeTime, const int deadAge,const int eta);
	~CSOINN();
	bool	InputSignal(const double *signal);
	void	Classify(void);
	void	Reset(const int dimension=NO_CHANGE, const int removeNodeTime=NO_CHANGE, const int deadAge=NO_CHANGE, const int eta=NO_CHANGE);
	bool	SetDimension(int dimension);
	int		GetDimension(void);
	int		GetNodeNum(const bool ignoreAcnode=false);
	int		GetEdgeNum(void);
	int		GetClassNum(void);
	CNode*	GetNode(const int node);
	CEdge*	GetEdge(const int edge);
	int     GetClassFromNode(const int node);
	void	RemoveUnnecessaryNode(void);
	bool	OutputNetworkData(string fileName);
	bool    LoadNetworkData(string fileName);

	double	GetNeighborMaxDist(const int node);
	bool	FindWinner(int &winner, double *minDist, const double *signal);
	bool	FindCondWinner(int &winner, int &learningTime, double *minDist, const double *signal, int neighborMinNum, int learningMinTime);
	int     GetLearningMaxTime();
	bool	RemoveNode(int node);
	bool	FindWinnerAndSecondWinner(int &winner, int &secondWinner, const double *signal);
	bool	IsEffectiveInput(const int winner, const int secondWinner, const double *signal);

	bool	IncrementEdgeAge(const int node);
	bool	ResetEdgeAge(const int node1, const int node2);
	bool	RemoveDeadEdge(void);
	bool	UpdateLearningTime(const int node);
	bool	MoveNode(const int node, const double *signal);
	double	Distance(const double *signal1, const double *signal2);
	double	Distance(const int node1, const int node2);
	double	GetSimilarityThreshold(const int node);
	bool	AddNode(const double *signal);
	bool	IsExistNode(int node);
	bool	AddEdge(const int node1, const int node2);
	bool	RemoveEdge(const int edge);
	bool	RemoveEdge(const int node1, const int node2);
	bool	IsExistEdge(const int edge);
	bool	IsExistEdge(const int node1, const int node2);
	int		FindEdge(const int node1, const int node2);
	bool	SetClassID(const int node, const int classID);

	bool AddNode(const double *signal,soinnNodeInfoData info);
	bool InputSignal(const double *signal,soinnNodeInfoData info);
	bool AddInfo(const int node,soinnNodeInfoData info);

private:
	std::vector<CNode*>	m_nodeInfo;
	std::vector<CEdge>	m_edgeInfo;
	int	m_dimension;
	int	m_removeNodeTime;
	int	m_deadAge;
	int	m_classNum;
	int m_eta;
	int effectiveInputNum;
};

#endif
