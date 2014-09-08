/////////////////////////////
// SOINN Version 3.02      //
// Managed by Daiki Kimura //
/////////////////////////////

#include "SOINN.h"
#include <math.h>

#define MY_ERROR(x)   printf("error:%s",x)
#define MY_WARNING(x) printf("warning:%s",x)
#define MY_INFO(x)    printf("information:%s",x)
#define MY_YESNO(x)   printf("YesNo:%s",x)

const int    CSOINN::NO_CHANGE    = 0;
const int    CSOINN::UNCLASSIFIED = -1;
const int    CSOINN::NOT_FOUND    = -1;
const double CSOINN::INFI    = 1e10;

const int CNode::MAX_NEIGHBOR = 30;
const int CNode::EMPTY        = -1;

const int CEdge::EMPTY = -1;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CEdge::CEdge()
{
	Init();
}

// Generate edge between the given nodes
CEdge::CEdge(const int node1, const int node2)
{
	Init();
	m_from = node1;
	m_to   = node2;
}


CEdge::~CEdge()
{
}


void CEdge::Init(void)
{
	m_from = EMPTY;
	m_to   = EMPTY;
	m_age  = 0;
}

// Copy edge
void CEdge::Copy(const CEdge &edge)
{
	::memcpy(this, &edge, sizeof(CEdge));
}

// Replace edge with new node
bool CEdge::Replace(const int before, const int after)
{
	if (before == after) return false;
	if (m_from == before && m_to == after) return false;
	if (m_to == before && m_from == after) return false;

	if (m_from == before) m_from = after;
	if (m_to   == before) m_to   = after;

	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CNode::CNode(int dimension)
{
	Init(dimension);
}

CNode::CNode(const double* signal,int dimension,soinnNodeInfoData infoInput)
{
	Init(dimension);

	if (m_dimension <= 0) return;
	if (m_signal == NULL)
	{
		m_signal = new double[m_dimension];
	}
	for(int i=0;i<m_dimension;i++) {
		m_signal[i]=signal[i];
	}

	infos.push_back(infoInput);
}

// Generate node with given data vector
CNode::CNode(const double* signal,int dimension)
{
	Init(dimension);

	if (m_dimension <= 0) return;
	if (m_signal == NULL)
	{
		m_signal = new double[m_dimension];
	}
	for(int i=0;i<m_dimension;i++) {
		m_signal[i]=signal[i];
	}
}

// The destructor frees the memory
CNode::~CNode()
{
	delete [] m_signal;

	while (!m_neighbor.empty())
	{
		m_neighbor.pop_back();
	}

	while(!infos.empty())
		infos.pop_back();
}

// The node is registered as neighbor node
// 
bool CNode::AddNeighbor(const int node)
{
	int i;

	// If number of neighbors is greater than threshold, return false
	// no limit!
	//if (m_neighborNum >= MAX_NEIGHBOR) return false;

	// If this node is already neighbor
	for (i=0; i<m_neighborNum; i++)
	{
		if (m_neighbor[i] == node) return false;
	}

	// Add this node as new neighbor
	// no limit!
	m_neighbor.push_back(node);	//m_neighbor[m_neighborNum] = node;
	m_neighborNum++;

	return true;
}

// Delete the neighbor node
bool CNode::DeleteNeighbor(const int node)	
{
	int i;

	for (i=0; i<m_neighborNum; i++)
	{
		if(node == m_neighbor[i])
		{
			m_neighborNum--;
			m_neighbor[i] = m_neighbor[m_neighborNum];
			m_neighbor.pop_back();	//m_neighbor[m_neighborNum] = EMPTY;
			return true;
		}
	}

	return false;
}

// Replace neighbor node
bool CNode::ReplaceNeighbor(const int before, const int after)
{
	int i;

	// If the nodes are same, return false
	if (before == after) return false;

	for (i=0; i<m_neighborNum; i++)
	{
		// If before node is neighbor node
		if (before == m_neighbor[i])
		{
			// If after node is also neighbor node
			if (IsNeighbor(after))
			{
				// remove before node
				m_neighborNum--;
				m_neighbor[i] = m_neighbor[m_neighborNum];
				m_neighbor.pop_back();	//m_neighbor[m_neighborNum] = EMPTY;
			}
			else
			{
				// replace before node with after node
				m_neighbor[i] = after;
			}
			return true;
		}
	}

	// If cannot find the before node
	return false;
}

// Judge if the node is the neighbor
bool CNode::IsNeighbor(const int node)
{
	int i;

	for (i=0; i<m_neighborNum; i++)
	{
		if (m_neighbor[i] == node)
		{
			return true;
		}
	}

	return false;
}

// get signal vector
double* CNode::GetSignal()
{
	return m_signal;
}

// get class id
int CNode::GetClass()
{
	return m_classID;
}

// get isNodeRemoved
bool CNode::GetIsNodeRemoved()
{
	return m_isNodeRemoved;
}

// set isNodeRemoved
void CNode::SetIsNodeRemoved(const bool isNodeRemoved)
{
	m_isNodeRemoved = isNodeRemoved;
}

// get isEdgeRemoved
bool CNode::GetIsEdgeRemoved()
{
	return m_isEdgeRemoved;
}

// set isEdgeRemoved
void CNode::SetIsEdgeRemoved(const bool isEdgeRemoved)
{
	m_isEdgeRemoved = isEdgeRemoved;
}

// Initialize
void CNode::Init(int dimension)
{
	m_dimension=dimension;

	if (m_dimension > 0)
	{
		m_signal       = new double[m_dimension];
	}
	m_neighbor.clear();	//m_neighbor     = new int[MAX_NEIGHBOR];  
	m_neighborNum  = 0;
	m_learningTime = 0;
	m_classID      = CSOINN::UNCLASSIFIED;

	while(!infos.empty())
		infos.pop_back();

	/*for (i=0; i<MAX_NEIGHBOR; i++)
	{
	m_neighbor[i] = EMPTY;
	}*/
}

// Copy node
void CNode::Copy(const CNode &src)
{
	if (m_signal != NULL)
	{
		delete [] m_signal;
	}
	//if (m_neighbor != NULL)
	//{
	//	delete [] m_neighbor;
	//}
	m_neighbor.clear();

	::memcpy(this, &src, sizeof(CNode));

	m_signal    = new double[m_dimension];
	//m_neighbor  = new int[MAX_NEIGHBOR];

	memcpy(m_signal,   src.m_signal,   sizeof(double)*m_dimension);
	//memcpy(m_neighbor, src.m_neighbor, sizeof(int)*MAX_NEIGHBOR);
	copy(src.m_neighbor.begin(), src.m_neighbor.end(), m_neighbor.begin());
}

// added for MAT-SOINN
int	CNode::GetNeighborNum()
{
	return m_neighborNum;
}

int CNode::GetLearningTime()
{
	return m_learningTime;
}

void CNode::AddInfo(soinnNodeInfoData infoInput)
{
	infos.push_back(infoInput);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Arguments: Dimension of input data, training time for removing node, threshold of age for removing edge
CSOINN::CSOINN(const int dimension, const int removeNodeTime, const int deadAge,const int eta)
{
	effectiveInputNum=0;

	// Initialize dimension
	if (dimension > 0)
	{
		m_dimension = dimension;
	}
	// Initialize training time for removing node
	if (removeNodeTime > 0)
	{
		m_removeNodeTime = removeNodeTime;
	}
	// Initialize threshold of age for removing edge
	if (deadAge > 0)
	{
		m_deadAge = deadAge;
	}
	if (eta > 0)
	{
		m_eta = eta;
	}
}

CSOINN::~CSOINN()
{
	int nodeNum=m_nodeInfo.size();
	for(int i=0;i<nodeNum;i++) {
		delete m_nodeInfo[i];
	}
	// Free memory and exit
	while (!m_nodeInfo.empty())
	{
		m_nodeInfo.pop_back();
	}
	while (!m_edgeInfo.empty())
	{
		m_edgeInfo.pop_back();
	}
}

bool CSOINN::InputSignal(const double *signal,soinnNodeInfoData info)
{
	//static int effectiveInputNum = 0; -> member
	int winner, secondWinner;

	// If the input data is invalid value, returan false and exit
	if (signal == NULL) return false;

	// If the input data belongs to known knowledge, count up the number of input data
	effectiveInputNum++;

	// If number of nodes is less than 2, directly add the input data as new node
	if (m_nodeInfo.size() < 2)
	{
		AddNode(signal,info);
		return true;
	}

	// Find winner and runnerup
	FindWinnerAndSecondWinner(winner, secondWinner, signal);

	// If the input data belongs to new knowledge, insert it as new node
	if (!IsEffectiveInput(winner, secondWinner, signal))
	{
		AddNode(signal,info);
	} else {
		AddEdge(winner, secondWinner);			// generate edge between winner and runnerup
		ResetEdgeAge(winner, secondWinner);		// Reset the age of edge between winner and runnerup
		IncrementEdgeAge(winner);				// Count up the age of edges linked with winner
		RemoveDeadEdge();						// Remove edges whose age are greater than threshold of age
		UpdateLearningTime(winner);				// Count up the number of times been winner
		MoveNode(winner, signal);				// Update weight of winner and neighbor of winner
		AddInfo(winner,info);
	}

	// If the learning times are greater than threshold, remove isolated nodes
	if (effectiveInputNum%m_removeNodeTime == 0)
	{
		RemoveUnnecessaryNode();
	}

	return true;
}


// Input data vector
bool CSOINN::InputSignal(const double *signal)
{
	//static int effectiveInputNum = 0; -> member
	int winner, secondWinner;

	// If the input data is invalid value, returan false and exit
	if (signal == NULL) return false;

	// If the input data belongs to known knowledge, count up the number of input data
	effectiveInputNum++;

	// If number of nodes is less than 2, directly add the input data as new node
	if (m_nodeInfo.size() < 2)
	{
		AddNode(signal);
		return true;
	}

	// Find winner and runnerup
	FindWinnerAndSecondWinner(winner, secondWinner, signal);

	// If the input data belongs to new knowledge, insert it as new node
	if (!IsEffectiveInput(winner, secondWinner, signal))
	{
		AddNode(signal);
	} else {

		AddEdge(winner, secondWinner);			// generate edge between winner and runnerup
		ResetEdgeAge(winner, secondWinner);		// Reset the age of edge between winner and runnerup
		IncrementEdgeAge(winner);				// Count up the age of edges linked with winner
		RemoveDeadEdge();						// Remove edges whose age are greater than threshold of age
		UpdateLearningTime(winner);				// Count up the number of times been winner
		MoveNode(winner, signal);				// Update weight of winner and neighbor of winner
	}

	// If the learning times are greater than threshold, remove isolated nodes
	if (effectiveInputNum%m_removeNodeTime == 0)
	{
		RemoveUnnecessaryNode();
	}

	return true;
}

// Label nodes with class label
void CSOINN::Classify(void)
{
	int i, nodeNum, classNum;

	nodeNum = (int)m_nodeInfo.size();
	for (i=0; i<nodeNum; i++)
	{
		// At first set all nodes as unlabeled
		m_nodeInfo[i]->m_classID = UNCLASSIFIED;
	}

	classNum = 0;
	for (i=0; i<nodeNum; i++)
	{
		if (m_nodeInfo[i]->m_classID == UNCLASSIFIED)
		{
			// If there are unlabeled nodes, label them
			// Recurrently call SetClassID(...)
			SetClassID(i, classNum);
			// If nodes are labeled with new class label, count up the number of classes
			classNum++;
		}
	}

	m_classNum = classNum;
}

// Reset SOINN
void CSOINN::Reset(const int dimension/*=NO_CHANGE*/, const int removeNodeTime/*=NO_CHANGE*/, const int deadAge/*=NO_CHANGE*/,const int eta/*=NO_CHANGE*/)
{
	int size = m_nodeInfo.size();
	for(int i=0;i<size;i++) {
		m_nodeInfo[i]->~CNode();
	}
	while (!m_nodeInfo.empty())
	{
		m_nodeInfo.pop_back();
	}
	while (!m_edgeInfo.empty())
	{
		m_edgeInfo.pop_back();
	}
	//m_nodeInfo.clear();
	//m_edgeInfo.clear();

	m_classNum = 0;

	if (dimension != NO_CHANGE && dimension > 0)
	{
		m_dimension      = dimension;
	}
	if (removeNodeTime != NO_CHANGE && removeNodeTime > 0)
	{
		m_removeNodeTime = removeNodeTime;
	}
	if (deadAge != NO_CHANGE && deadAge > 0)
	{
		m_deadAge        = deadAge;
	}
	if (deadAge != NO_CHANGE && eta > 0)
	{
		m_eta           = eta;
	}
}

bool CSOINN::SetDimension(int dimension)
{
	if (m_dimension == dimension) return false;
	if (dimension <= 0) return false;

	Reset(dimension, NO_CHANGE, NO_CHANGE);

	return true;
}

int CSOINN::GetDimension(void)
{
	return m_dimension;
}

int CSOINN::GetNodeNum(const bool ignoreAcnode/*=false*/)
{
	int i, nodeNum, count;

	if (ignoreAcnode)
	{
		// Return the number of nodes who are not isolated node
		count = 0;
		nodeNum = (int)m_nodeInfo.size();
		for (i=0; i<nodeNum; i++)
		{
			if (m_nodeInfo[i]->m_neighborNum > 0)
			{
				count++;
			}
		}
		return count;
	}
	else
	{
		// Return number of all nodes
		return (int)m_nodeInfo.size();
	}
}

int CSOINN::GetEdgeNum(void)
{
	return (int)m_edgeInfo.size();
}

int CSOINN::GetClassNum(void)
{
	return m_classNum;
}

CNode* CSOINN::GetNode(const int node)
{
	if (!IsExistNode(node)) return NULL;

	return m_nodeInfo[node];
}

CEdge* CSOINN::GetEdge(const int edge)
{
	if (!IsExistEdge(edge)) return NULL;

	return &(m_edgeInfo[edge]);
}

int CSOINN::GetClassFromNode(const int node)
{
	if (!IsExistNode(node)) return NOT_FOUND;

	return m_nodeInfo[node]->m_classID;
}

// Find winner and runnerup
// 
bool CSOINN::FindWinnerAndSecondWinner(int &winner, int &secondWinner, const double *signal)
{
	int i, nodeNum;
	double dist, minDist, secondMinDist;

	winner        = NOT_FOUND;
	secondWinner  = NOT_FOUND;

	// If number of nodes is less than 2, return false and exit
	if (m_nodeInfo.size() < 2) return false;

	minDist       = INFI;
	secondMinDist = INFI;

	nodeNum = (int)m_nodeInfo.size();
	for (i=0; i<nodeNum; i++)
	{
		// Calculate distance
		dist = Distance(m_nodeInfo[i]->m_signal, signal);
		if (minDist > dist)
		{
			secondMinDist = minDist;
			minDist       = dist;
			secondWinner  = winner;
			winner        = i;
		}
		else if (secondMinDist > dist)
		{
			secondMinDist = dist;
			secondWinner  = i;
		}
	}

	return true;
}

// Judge if the input data belongs to known knowledge or new knowledge
bool CSOINN::IsEffectiveInput(const int winner, const int secondWinner, const double *signal)
{
	if (!IsExistNode(winner)) return false;
	if (!IsExistNode(secondWinner)) return false;

	if (Distance(m_nodeInfo[winner]->m_signal, signal) > GetSimilarityThreshold(winner))
	{
		return false;
	}
	if (Distance(m_nodeInfo[secondWinner]->m_signal, signal) > GetSimilarityThreshold(secondWinner))
	{
		return false;
	}

	// If distance between input data and winner or runnerup is less than similarity threshold, it belongs to known knowledge
	return true;
}

// Increment the age of edge
bool CSOINN::IncrementEdgeAge(const int node)
{
	int i, f, t, edgeNum;

	// If no such nodes exited, return false and exit
	if (!IsExistNode(node)) return false;
	// If there is no neighbor of this node, return false and exit
	if (m_nodeInfo[node]->m_neighborNum == 0) return false;

	// Count up the age of edge linked with the node
	edgeNum = (int)m_edgeInfo.size();
	for (i=0; i<edgeNum; i++)
	{
		f = m_edgeInfo[i].m_from;
		t = m_edgeInfo[i].m_to;
		if (f == node || t == node)
		{
			m_edgeInfo[i].m_age++;
		}
	}

	return true;
}

bool CSOINN::ResetEdgeAge(const int node1, const int node2)
{
	int edge;

	if (node1 == node2) return false;
	if (!IsExistNode(node1)) return false;
	if (!IsExistNode(node2)) return false;

	edge = FindEdge(node1, node2);
	if (edge == NOT_FOUND) return false;

	m_edgeInfo[edge].m_age = 0;

	return true;
}

// Remove edges whose age are greater than threshold age
// Return true if there is removed edge
bool CSOINN::RemoveDeadEdge(void)
{
	bool isRemoved = false;
	int lastEdge = ((int)m_edgeInfo.size() - 1);

	for (int i = 0; i < (int) m_nodeInfo.size(); i++)
	{
		m_nodeInfo[i]->SetIsEdgeRemoved(false);
		m_nodeInfo[i]->SetIsNodeRemoved(false);
	}

	for (int i = lastEdge; i >= 0; i--)
	{
		if (m_edgeInfo[i].m_age > m_deadAge)
		{
			m_nodeInfo[m_edgeInfo[i].m_from]->SetIsEdgeRemoved(true);
			m_nodeInfo[m_edgeInfo[i].m_to]->SetIsEdgeRemoved(true);

			RemoveEdge(i);
			isRemoved = true;
		}
	}

	int lastNode = ((int)m_nodeInfo.size() - 1);
	for (int i = lastNode; i >= 0; i--)
	{
		if (m_nodeInfo[i]->GetIsEdgeRemoved() && m_nodeInfo[i]->m_neighborNum == 0)
		{
			m_nodeInfo[i]->SetIsNodeRemoved(true);
		}
	}

	for (int i = lastNode; i >= 0; i--)
	{
		if (m_nodeInfo[i]->GetIsNodeRemoved())
		{
			RemoveNode(i);
		}
	}

	return isRemoved;
}

// Update the times been winner for the node
bool CSOINN::UpdateLearningTime(const int node)
{
	if (!IsExistNode(node)) return false;

	m_nodeInfo[node]->m_learningTime++;

	return true;
}

// update the weight of node and its neighbor
bool CSOINN::MoveNode(const int node, const double *signal)
{
	int i, j, neighbor, neighborNum;
	double learningRateOfNode, learningRateOfNeighbor;

	if (!IsExistNode(node)) return false;
	if (m_nodeInfo[node]->m_learningTime == 0) return false;

	learningRateOfNode     = 1.0/(double)m_nodeInfo[node]->m_learningTime;
	learningRateOfNeighbor = learningRateOfNode/100.0;
	//	learningRateOfNeighbor = 0.0; // If want to converge to average position

	for (j=0; j<m_dimension; j++)
	{
		m_nodeInfo[node]->m_signal[j] += learningRateOfNode*(signal[j]-m_nodeInfo[node]->m_signal[j]);
	}

	neighborNum = m_nodeInfo[node]->m_neighborNum;
	for (i=0; i<neighborNum; i++)
	{
		neighbor = m_nodeInfo[node]->m_neighbor[i];
		for (j=0; j<m_dimension; j++)
		{
			m_nodeInfo[neighbor]->m_signal[j] += learningRateOfNeighbor*(signal[j]-m_nodeInfo[neighbor]->m_signal[j]);
		}
	}

	return true;
}

// Remove isolated nodes when learning time is greater than threshold
void CSOINN::RemoveUnnecessaryNode(void)
{
	int lastNode = ((int) m_nodeInfo.size() - 1);
	for (int i = lastNode; i >= 0; i--)
	{
		if (m_nodeInfo[i]->m_neighborNum < m_eta)
		{
			m_nodeInfo[i]->SetIsNodeRemoved(true);
		}
		else
		{
			m_nodeInfo[i]->SetIsNodeRemoved(false);
		}
	}

	for (int i = lastNode; i >= 0; i--)
	{
		if (m_nodeInfo[i]->GetIsNodeRemoved())
		{
			RemoveNode(i);
		}
	}
}

// Output the network data of SOINN
// LoadNetworkData(...) can be used to restore the network for learning
/*
bool CSOINN::OutputNetworkData(string fileName)
{
	HANDLE hFile;
	DWORD dwWriteSize;
	int i, nodeNum, edgeNum;

	hFile = CreateFile(fileName.c_str(), GENERIC_WRITE, FILE_SHARE_WRITE, NULL, CREATE_ALWAYS, 0, NULL);
	if (hFile == INVALID_HANDLE_VALUE) return false;

	WriteFile(hFile, &(this->m_dimension), sizeof(int), &dwWriteSize, NULL);
	WriteFile(hFile, &(this->m_removeNodeTime), sizeof(int), &dwWriteSize, NULL);
	WriteFile(hFile, &(this->m_deadAge), sizeof(int), &dwWriteSize, NULL);
	WriteFile(hFile, &(this->m_eta), sizeof(int), &dwWriteSize, NULL);
	WriteFile(hFile, &(this->m_classNum), sizeof(int), &dwWriteSize, NULL);

	nodeNum = (int)m_nodeInfo.size();
	edgeNum = (int)m_edgeInfo.size();
	WriteFile(hFile, &nodeNum, sizeof(int), &dwWriteSize, NULL);
	for (i=0; i<nodeNum; i++)
	{
		WriteFile(hFile, m_nodeInfo[i]->m_signal, sizeof(double)*m_dimension, &dwWriteSize, NULL);
		WriteFile(hFile, &(m_nodeInfo[i]->m_neighborNum), sizeof(int), &dwWriteSize, NULL);

		int num = m_nodeInfo[i]->m_neighborNum;
		int *tmp = new int[m_nodeInfo[i]->m_neighborNum];
		for ( int j=0; j<num; j++ )
			tmp[j] = m_nodeInfo[i]->m_neighbor[j];
		WriteFile(hFile, tmp, sizeof(int)*m_nodeInfo[i]->m_neighborNum, &dwWriteSize, NULL);
		delete tmp;

		WriteFile(hFile, &(m_nodeInfo[i]->m_learningTime), sizeof(int), &dwWriteSize, NULL);
		WriteFile(hFile, &(m_nodeInfo[i]->m_classID), sizeof(int), &dwWriteSize, NULL);

		int infoSize=m_nodeInfo[i]->infos.size();
		WriteFile(hFile, &(infoSize), sizeof(int), &dwWriteSize, NULL);
		for(int j=0;j<infoSize;j++) {
			WriteFile(hFile, &(m_nodeInfo[i]->infos[j].fileName), sizeof(int), &dwWriteSize, NULL);
			WriteFile(hFile, &(m_nodeInfo[i]->infos[j].cannyTh1), sizeof(int), &dwWriteSize, NULL);
			WriteFile(hFile, &(m_nodeInfo[i]->infos[j].cannyTh2), sizeof(int), &dwWriteSize, NULL);
			WriteFile(hFile, &(m_nodeInfo[i]->infos[j].x), sizeof(int), &dwWriteSize, NULL);
			WriteFile(hFile, &(m_nodeInfo[i]->infos[j].y), sizeof(int), &dwWriteSize, NULL);
			WriteFile(hFile, &(m_nodeInfo[i]->infos[j].width), sizeof(int), &dwWriteSize, NULL);
			WriteFile(hFile, &(m_nodeInfo[i]->infos[j].height), sizeof(int), &dwWriteSize, NULL);
			WriteFile(hFile, &(m_nodeInfo[i]->infos[j].firstImageClass), sizeof(string), &dwWriteSize, NULL);
		}

	}
	WriteFile(hFile, &edgeNum, sizeof(int), &dwWriteSize, NULL);
	for (i=0; i<edgeNum; i++)
	{
		WriteFile(hFile, &m_edgeInfo[i].m_from, sizeof(int), &dwWriteSize, NULL);
		WriteFile(hFile, &m_edgeInfo[i].m_to,   sizeof(int), &dwWriteSize, NULL);
		WriteFile(hFile, &m_edgeInfo[i].m_age, sizeof(int), &dwWriteSize, NULL);
	}
	CloseHandle(hFile);
	return true;
}*/

double CSOINN::Distance(const double *signal1, const double *signal2)
{
	int i;
	double sum;

	if (signal1 == NULL || signal2 == NULL) return 0.0;

	sum = 0.0;
	for (i=0; i<m_dimension; i++)
	{
		sum += (signal1[i]-signal2[i])*(signal1[i]-signal2[i]);
	}

	return sqrt(sum)/(double)m_dimension;
}

double CSOINN::Distance(const int node1, const int node2)
{
	if (node1 == node2) return 0.0;
	if (!IsExistNode(node1)) return 0.0;
	if (!IsExistNode(node2)) return 0.0;

	return Distance(m_nodeInfo[node1]->m_signal, m_nodeInfo[node2]->m_signal);
}

// Calculate the simiarity threshold
double CSOINN::GetSimilarityThreshold(const int node)
{
	int i, nodeNum, neighborNum, neighbor;
	double dist, minDist, maxDist;

	// If the node is not exist, return 0
	if (!IsExistNode(node)) return 0.0;

	// Get the number of neighbors
	neighborNum = m_nodeInfo[node]->m_neighborNum;
	if (neighborNum > 0)
	{
		// If there are neighbor nodes
		maxDist = 0.0;
		for (i=0; i<neighborNum; i++)
		{
			// Set the longest distance between neighbors and the node as the similarity threshold
			neighbor = m_nodeInfo[node]->m_neighbor[i];
			dist = Distance(node, neighbor);
			if (maxDist < dist)
			{
				maxDist = dist;
			}
		}

		return maxDist;
	}
	else
	{
		// If there is no neighbor
		minDist = INFI;
		nodeNum = (int)m_nodeInfo.size();
		for (i=0; i<nodeNum; i++)
		{
			// Set the shortest distance of all nodes to the node as the similarity threshold
			if (i != node)
			{
				dist = Distance(node, i);
				if (minDist > dist)
				{
					minDist = dist;
				}
			}
		}

		return minDist;
	}
}

double CSOINN::GetNeighborMaxDist(const int node)
{
	int i, neighborNum, neighbor;
	double dist, maxDist;

	// If the node is not exist, return 0
	if (!IsExistNode(node)) return 0.0;

	// Get the number of neighbors
	neighborNum = m_nodeInfo[node]->m_neighborNum;
	if (neighborNum > 0)
	{
		// If there are neighbor nodes
		maxDist = 0.0;
		for (i=0; i<neighborNum; i++)
		{
			// Set the longest distance between neighbors and the node as the similarity threshold
			neighbor = m_nodeInfo[node]->m_neighbor[i];
			dist = Distance(node, neighbor);
			if (maxDist < dist)
			{
				maxDist = dist;
			}
		}

		return maxDist;
	} else 
		return 0.0;
}

bool CSOINN::AddNode(const double *signal,soinnNodeInfoData info)
{
	CNode *newNode=new CNode(signal, this->m_dimension,info);
	m_nodeInfo.push_back(newNode);

	return true;
}

bool CSOINN::AddNode(const double *signal)
{
	CNode *newNode=new CNode(signal, this->m_dimension);
	m_nodeInfo.push_back(newNode);

	return true;
}

bool CSOINN::RemoveNode(int node)
{
	int i, nodeNum, edgeNum, lastNode;

	if (!IsExistNode(node)) return false;

	//while (m_nodeInfo[node]->m_neighborNum > 0)
	//{
	//	neighbor = m_nodeInfo[node]->m_neighbor[0];
	//	RemoveEdge(node, neighbor);
	//}

	nodeNum  = (int)m_nodeInfo.size();
	lastNode = (int)m_nodeInfo.size()-1;

	int thisNodeEdghNum=m_nodeInfo[node]->m_neighborNum;
	for (i=0; i<thisNodeEdghNum; i++)
	{
		RemoveEdge(node,m_nodeInfo[node]->m_neighbor[0]);
	}

	edgeNum  = (int)m_edgeInfo.size();

	if (node < lastNode)
	{
		for (i=0; i<nodeNum; i++)
		{
			if(i!=node&&i!=lastNode)
				m_nodeInfo[i]->ReplaceNeighbor(lastNode, node);
		}
		for (i=0; i<edgeNum; i++)
		{
			m_edgeInfo[i].Replace(lastNode, node);
		}
		m_nodeInfo[node]->~CNode();
		m_nodeInfo[node] = m_nodeInfo[lastNode];
	} else {
		m_nodeInfo[node]->~CNode();
	}
	m_nodeInfo.pop_back();

	return true;
}

// Judge if the node exist or not
bool CSOINN::IsExistNode(int node)
{
	if (node < 0)          return false;
	if (node >= (int)m_nodeInfo.size()) return false;

	return true;
}

bool CSOINN::AddEdge(const int node1, const int node2)
{
	if (node1 == node2) return false;
	if (!IsExistNode(node1)) return false;
	if (!IsExistNode(node2)) return false;
	if (IsExistEdge(node1, node2)) return false;

	CEdge newEdge(node1, node2);
	m_edgeInfo.push_back(newEdge);
	m_nodeInfo[node1]->AddNeighbor(node2);
	m_nodeInfo[node2]->AddNeighbor(node1);

	return true;
}

bool CSOINN::RemoveEdge(const int edge)
{
	int f, t, lastEdge;

	if (!IsExistEdge(edge)) return false;

	f = m_edgeInfo[edge].m_from;
	t = m_edgeInfo[edge].m_to;

	m_nodeInfo[f]->DeleteNeighbor(t);
	m_nodeInfo[t]->DeleteNeighbor(f);

	lastEdge = (int)m_edgeInfo.size()-1;
	if (edge < lastEdge)
	{
		m_edgeInfo[edge] = m_edgeInfo[lastEdge];
	}
	m_edgeInfo.pop_back();

	return true;
}

bool CSOINN::RemoveEdge(const int node1, const int node2)
{
	int edge;

	if (node1 == node2) return false;
	if (!IsExistNode(node1)) return false;
	if (!IsExistNode(node2)) return false;

	edge = FindEdge(node1, node2);
	if (edge == NOT_FOUND) return false;

	return RemoveEdge(edge);
}

bool CSOINN::IsExistEdge(const int edge)
{
	if (edge < 0 || (int)m_edgeInfo.size() <= edge) return false;

	return true;
}

bool CSOINN::IsExistEdge(const int node1, const int node2)
{
	int i, f, t, edgeNum;

	if (node1 == node2) return false;
	if (!IsExistNode(node1)) return false;
	if (!IsExistNode(node2)) return false;

	edgeNum = (int)m_edgeInfo.size();
	for (i=0; i<edgeNum; i++)
	{
		f = m_edgeInfo[i].m_from;
		t = m_edgeInfo[i].m_to;
		if ((f == node1 && t == node2) || (t == node1 && f == node2))
		{
			return true;
		}
	}

	return false;
}

int CSOINN::FindEdge(const int node1, const int node2)
{
	int i, f, t, edgeNum;

	if (node1 == node2) return false;
	if (!IsExistNode(node1)) return false;
	if (!IsExistNode(node2)) return false;

	edgeNum = (int)m_edgeInfo.size();
	for (i=0; i<edgeNum; i++)
	{
		f = m_edgeInfo[i].m_from;
		t = m_edgeInfo[i].m_to;
		if ((f == node1 && t == node2) || (t == node1 && f == node2))
		{
			return i;
		}
	}

	return NOT_FOUND;
}

// Label the node with classID, and then label all nodes linked with the node recurrently
bool CSOINN::SetClassID(const int node, const int classID)
{
	int i, neighbor, neighborNum;

	if (!IsExistNode(node)) return false;
	if (m_nodeInfo[node]->m_classID != UNCLASSIFIED) return false;

	m_nodeInfo[node]->m_classID = classID;
	neighborNum = m_nodeInfo[node]->m_neighborNum;
	for (i=0; i<neighborNum; i++)
	{
		neighbor = m_nodeInfo[node]->m_neighbor[i];
		if (m_nodeInfo[neighbor]->m_classID == UNCLASSIFIED)
		{
			SetClassID(neighbor, classID);
		}
	}

	return true;
}
/*
bool CSOINN::LoadNetworkData(string fileName)
{
	HANDLE hFile;
	DWORD dwWriteSize;
	int i, nodeNum, edgeNum;
	int intTemp;

	hFile = CreateFile(fileName.c_str(), GENERIC_READ, NULL, NULL, OPEN_EXISTING, NULL, NULL);
	if (hFile == INVALID_HANDLE_VALUE){
		MY_INFO(TEXT("cannot open the file to load"));
		return false;
	}


	// SOINN data
	ReadFile(hFile, &intTemp, sizeof(int), &dwWriteSize, NULL);
	if(intTemp != this->m_dimension){
		if(MY_YESNO(TEXT("m_dimension is NOT equal to loaded data\ndo you want to overwrite?")) == IDNO){
			MY_INFO(TEXT("abort loading"));
			return false;
		}
	}
	this->m_dimension = intTemp;

	ReadFile(hFile, &intTemp, sizeof(int), &dwWriteSize, NULL);
	if(intTemp != this->m_removeNodeTime){
		if(MY_YESNO(TEXT("m_removeNodeTime is NOT equal to loaded data\ndo you want to overwrite?")) == IDNO){
			MY_INFO(TEXT("abort loading"));
			return false;
		}
	}
	this->m_removeNodeTime = intTemp;

	ReadFile(hFile, &intTemp, sizeof(int), &dwWriteSize, NULL);
	if(intTemp != this->m_deadAge){
		if(MY_YESNO(TEXT("m_deadAge is NOT equal to loaded data\ndo you want to overwrite?")) == IDNO){
			MY_INFO(TEXT("abort loading"));
			return false;
		}
	}
	this->m_deadAge = intTemp;

	ReadFile(hFile, &intTemp, sizeof(int), &dwWriteSize, NULL);
	if(intTemp != this->m_eta){
		if(MY_YESNO(TEXT("m_eta is NOT equal to loaded data\ndo you want to overwrite?")) == IDNO){
			MY_INFO(TEXT("abort loading"));
			return false;
		}
	}
	this->m_eta = intTemp;

	ReadFile(hFile, &(this->m_classNum), sizeof(int), &dwWriteSize, NULL);

	ReadFile(hFile, &nodeNum, sizeof(int), &dwWriteSize, NULL);
	for(i=0; i<nodeNum; i++){
		m_nodeInfo.push_back(new CNode(this->m_dimension));
	}
	for (i=0; i<nodeNum; i++)
	{
		ReadFile(hFile, this->m_nodeInfo[i]->m_signal, sizeof(double)*m_dimension, &dwWriteSize, NULL);
		ReadFile(hFile, &(this->m_nodeInfo[i]->m_neighborNum), sizeof(int), &dwWriteSize, NULL);
		int *tmp = new int[this->m_nodeInfo[i]->m_neighborNum];
		ReadFile(hFile, tmp, sizeof(int)*this->m_nodeInfo[i]->m_neighborNum, &dwWriteSize, NULL);
		this->m_nodeInfo[i]->m_neighbor.resize(this->m_nodeInfo[i]->m_neighborNum);
		copy(tmp,tmp+this->m_nodeInfo[i]->m_neighborNum,this->m_nodeInfo[i]->m_neighbor.begin());
		delete tmp;
		ReadFile(hFile, &(this->m_nodeInfo[i]->m_learningTime), sizeof(int), &dwWriteSize, NULL);
		ReadFile(hFile, &(this->m_nodeInfo[i]->m_classID), sizeof(int), &dwWriteSize, NULL);

		int infosSize;
		ReadFile(hFile, &(infosSize), sizeof(int), &dwWriteSize, NULL);
		for(int j=0;j<infosSize;j++) {
			soinnNodeInfoData tmpInfo;
			ReadFile(hFile, &(tmpInfo.fileName), sizeof(int), &dwWriteSize, NULL);
			ReadFile(hFile, &(tmpInfo.cannyTh1), sizeof(int), &dwWriteSize, NULL);
			ReadFile(hFile, &(tmpInfo.cannyTh2), sizeof(int), &dwWriteSize, NULL);
			ReadFile(hFile, &(tmpInfo.x), sizeof(int), &dwWriteSize, NULL);
			ReadFile(hFile, &(tmpInfo.y), sizeof(int), &dwWriteSize, NULL);
			ReadFile(hFile, &(tmpInfo.width), sizeof(int), &dwWriteSize, NULL);
			ReadFile(hFile, &(tmpInfo.height), sizeof(int), &dwWriteSize, NULL);
			ReadFile(hFile, &(tmpInfo.firstImageClass), sizeof(string), &dwWriteSize, NULL);
			this->m_nodeInfo[i]->infos.push_back(tmpInfo);
		}

	}

	ReadFile(hFile, &edgeNum, sizeof(int), &dwWriteSize, NULL);
	for(i=0; i<edgeNum; i++){
		m_edgeInfo.push_back(CEdge());
	}
	for (i=0; i<edgeNum; i++)
	{
		ReadFile(hFile, &m_edgeInfo[i].m_from, sizeof(int), &dwWriteSize, NULL);
		ReadFile(hFile, &m_edgeInfo[i].m_to,   sizeof(int), &dwWriteSize, NULL);
		ReadFile(hFile, &m_edgeInfo[i].m_age,   sizeof(int), &dwWriteSize, NULL);
	}
	CloseHandle(hFile);
	return true;
}*/

// Find winner
bool CSOINN::FindWinner(int &winner, double *minDist, const double *signal)
{
	int i, nodeNum;
	double dist;

	winner        = NOT_FOUND;

	// If number of nodes is less than 2, return false and exit
	if (m_nodeInfo.size() < 2) return false;

	*minDist       = INFI;

	nodeNum = (int)m_nodeInfo.size();
	for (i=0; i<nodeNum; i++)
	{
		// Calculate distance
		dist = Distance(m_nodeInfo[i]->m_signal, signal);
		if (*minDist > dist)
		{
			*minDist = dist;
			winner   = i;
		}
	}
	return true;
}

// Find winner with condition
bool CSOINN::FindCondWinner(int &winner, int &learningTime, double *minDist, const double *signal, int neighborMinNum, int learningMinTime)
{
	int i, nodeNum;
	double dist;

	winner        = NOT_FOUND;

	// If number of nodes is less than 2, return false and exit
	if (m_nodeInfo.size() < 2) return false;

	*minDist       = INFI;

	nodeNum = (int)m_nodeInfo.size();
	for (i=0; i<nodeNum; i++)
	{
		CNode *node = GetNode(i);
		int nodeLearningTime = node->GetLearningTime();
		if ( node->GetNeighborNum() > neighborMinNum && nodeLearningTime > learningMinTime ) {
			// Calculate distance
			dist = Distance(m_nodeInfo[i]->m_signal, signal);
			if (*minDist > dist)
			{
				*minDist = dist;
				learningTime = nodeLearningTime;
				winner   = i;
			}
		}
	}
	return true;
}

// Get learning max time
int CSOINN::GetLearningMaxTime()
{
	int learningMaxTime = 0;
	for ( int i=0; i<GetNodeNum(); i++ ) {
		int learningTime = m_nodeInfo[i]->GetLearningTime();
		if ( learningTime > learningMaxTime )
			learningMaxTime = learningTime;
	}
	return learningMaxTime;
}

bool CSOINN::AddInfo(int node,soinnNodeInfoData info)
{
	m_nodeInfo[node]->AddInfo(info);
	return true;
}