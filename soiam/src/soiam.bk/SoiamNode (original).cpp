#include "SoiamNode.h"
#include "SOINN.h"

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <ros/package.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "cvd/thread.h"

#include <sys/stat.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <cmath>


using namespace std;

pthread_mutex_t SoiamNode::send_CS = PTHREAD_MUTEX_INITIALIZER;
SoiamNode::SoiamNode()
{
   	pose_channel = nh_.resolveName("ardrone/predictedPose");
	control_channel	= nh_.resolveName("cmd_vel");
   	common_channel = nh_.resolveName("tum_ardrone/com");
	predict_channel	= nh_.resolveName("cmd_vel");
	learning_toggle = nh_.resolveName("soiam/train");
	predicting_toggle = nh_.resolveName("soiam/predict");
	//packagePath = ros::package::getPath("soiam");

	std::string val;
	float valFloat;
	ros::param::get("~minPublishFreq", val);
	if(val.size()>0)
		sscanf(val.c_str(), "%f", &valFloat);
	else
		valFloat = 110;
	minPublishFreq = valFloat;
	cout << "set minPublishFreq to " << valFloat << "ms"<< endl;

	// internal vars
	learningSet.xpos = learningSet.ypos = learningSet.xlin = learningSet.ylin = learningSet.zlin = learningSet.zang = 0;
	soiamCMD.gaz = soiamCMD.pitch = soiamCMD.roll = soiamCMD.yaw = 0;
	for(int b=0;b<Key;b++){
		Xbuf[b] = cXbuf[b] = Ybuf[b] = cYbuf[b] = 0;
	}
	isLearning = isPredicting = false;

	// channels - subs
	pose_sub	= nh_.subscribe(pose_channel, 10, &SoiamNode::poseCb, this);
	vel_sub		= nh_.subscribe(control_channel, 10, &SoiamNode::velCb, this);
	com_sub		= nh_.subscribe(common_channel, 50, &SoiamNode::comCb, this);
	isLearning_sub	= nh_.subscribe(learning_toggle, 50, &SoiamNode::learnCb, this);
	isPredict_sub	= nh_.subscribe(predicting_toggle, 50, &SoiamNode::predCb, this);

	// channels - pubs
	//com_pub	= nh_.advertise<std_msgs::String>(command_channel,50);
	predict_pub	= nh_.advertise<geometry_msgs::Twist>(predict_channel,1);

	// create soinn network
	csoinn = new CSOINN(Dim*Key,40000,850,1);
	zsoinn = new CSOINN(2*Key,40000,850,1);
}

SoiamNode::~SoiamNode()
{
	// don't delete ~ nothing here, as this is also called for shallow copy.
}

void SoiamNode::poseCb(const tum_ardrone::filter_stateConstPtr statePtr)
{
	learningSet.xpos = statePtr->x;
	learningSet.ypos = statePtr->y;
	//learningSet.zpos = statePtr->z;
	//learningSet.yaw = statePtr->yaw;
}	

void SoiamNode::velCb(const geometry_msgs::TwistConstPtr velPtr)
{
	geometry_msgs::TwistStamped ts;
	ts.header.stamp = ros::Time::now();
	ts.twist = *velPtr;

	learningSet.xlin = ts.twist.linear.x;
	learningSet.ylin = ts.twist.linear.y;
	learningSet.zlin = ts.twist.linear.z;
	learningSet.zang = ts.twist.angular.z;
	
	// assume that while actively controlling, the above for will never be equal to zero, so i will never hover.
	// learningSet.angular.x = learningSet.angular.y = 0;
}

void SoiamNode::comCb(const std_msgs::StringConstPtr str)
{
}

void SoiamNode::learnCb(const std_msgs::Bool::ConstPtr learn)
{																		
	isLearning = learn->data;
}

void SoiamNode::predCb(const std_msgs::Bool::ConstPtr pred)
{																		
	isPredicting = pred->data;
}

void SoiamNode::nodetoSoinn(CSOINN *csoinn, double *signal){

	const char learnItem[Dim*Key][6] = 
	{
		{"X1"},{"X2"},{"X3"},{"Y1"},{"Y2"},{"Y3"},
		{"cmdX1"},{"cmdX2"},{"cmdX3"},
		{"cmdY1"},{"cmdY2"},{"cmdY3"},
		{"cmdZ1"},{"cmdZ2"},{"cmdZ3"},
		{"cmdR1"},{"cmdR2"},{"cmdR3"}
	};

	std::cout << "Add new Node to SOINN..." << endl; // CP
	// add one row a time when this is called
	csoinn->InputSignal(signal);

	// Nodes NO. + Check point
	std::cout<<"Finished Training with "<< csoinn->GetNodeNum() <<" Nodes"<<endl; // CP: No. of Nodes
	rowsnode = csoinn->GetNodeNum();
	colsnode = Dim*Key;

	// Loop of CP
	cout << "Values of sonode : " << endl;
	for(int i=0; i<Dim*Key; i++) {
		cout<< learnItem[i] << ": " << signal[i] << " " <<endl; // CP
		//cout<< csoinn->GetNode(2)->GetSignal()[i] <<" "<<endl; // CP
	}
}

void SoiamNode::Loop()
{
	ros::Time last = ros::Time::now();
	ros::Time lastStateUpdate = ros::Time::now();
	loadTrainingdata(csoinn);

	while (nh_.ok())
	{
		// -------------- 1. spin for 50ms, do main controlling part here. ---------------
		while((ros::Time::now() - last) < ros::Duration(minPublishFreq / 1000.0))
			ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(minPublishFreq / 1000.0 - (ros::Time::now() - last).toSec()));
		last = ros::Time::now();

		// -------------- 2. train soinn. (maybe). do something good. (hope). ------------
		if(isLearning)
		{
			for(int b=0;b<Key-1;b++){
				Xbuf[b] = Xbuf[b+1];
				cXbuf[b] = cXbuf[b+1];
				Ybuf[b] = Ybuf[b+1];
				cYbuf[b] = cYbuf[b+1];
			}

			Xbuf[Key-1] = learningSet.xpos;
			cXbuf[Key-1] = learningSet.xlin;
			Ybuf[Key-1] = learningSet.ypos;
			cYbuf[Key-1] = learningSet.ylin;

			// Popout the signal set to train
			double onsignal[Dim*Key]; // Remove command: records with -CMD
			for(int f=0;f<Key;f++){
				onsignal[f] = Xbuf[f];			// f[0]	f[1] f[2]
				onsignal[f+3] = Ybuf[f];		// f[3] f[4] f[5]
				onsignal[f+6] = cXbuf[f];		// f[6] f[7] f[8]
				onsignal[f+9] = cYbuf[f];		// f[9]	f[10]f[11]
				onsignal[f+12] = 0;			// f[12]f[13]f[14]
				onsignal[f+15] = 0;			// f[15]f[16]f[17]
			}
			// Add node to soinn
			nodetoSoinn(csoinn, onsignal);
		}

		if(isPredicting)
		{
			for(int b=0;b<Key-2;b++){
				pXbuf[b] = pXbuf[b+1];
				pYbuf[b] = pYbuf[b+1];
			}

			pXbuf[Key-2] = learningSet.xpos;
			pYbuf[Key-2] = learningSet.ypos;

			// Popout the signal set to recall
			double Psignal[realDime];
			for(int f=0;f<Key-1;f++){
				Psignal[f] = pXbuf[f];			// f[0]	f[1]
				Psignal[f+2] = pYbuf[f];		// f[2] f[3]
			}
			// TODO:recall from SOIAM
			soiamFilter(csoinn, rowsnode, colsnode);
			soiamPrediction(csoinn, filteredNode, Psignal, rowsnode, colsnode);
		}

		recFile (learningSet.xpos, learningSet.ypos, learningSet.xlin, learningSet.ylin, learningSet.zlin, learningSet.zang);

		//cout<< isLearning <<endl; // CP
		// -------------- 2. update time. ---------------
		if((ros::Time::now() - lastStateUpdate) > ros::Duration(0.4))
		{
			lastStateUpdate = ros::Time::now();
		}
	}
}

void SoiamNode::soiamFilter(CSOINN *csoinn, int rowsnode, int colsnode) {
		/*
	for(int r=0; r<Dim-CMD ;r++){
		signal[r] = rescaling(signal[r], 0, 640, -1, 1);
		signal[r+2] = rescaling(signal[r+2], 0, 360, -1, 1);
	}*/
	// SOINN Nodes + Removal of command or last cols of each measure (if necessary)
	int s, n, sk;
	filteredNode = new double*[rowsnode];
	for(int i=0;i<rowsnode;i++) {
		//cout<<i<<" : ";
		filteredNode[i]=new double[colsnode-(CMD*Key)];
		n=1; s=0; sk=0;
		for(int j=0;j<colsnode-(CMD*Key);j++) { // Clear the last data cols for prediction(??)
			//cout<<csoinn->GetNode(i)->GetSignal()[j]<<" "; // CP
			if (j==(Key-1)*n+s){
				s++;
				n++;
				continue;
			}
			filteredNode[i][sk] = csoinn->GetNode(i)->GetSignal()[j];
			sk++;
		}
		//cout<<endl;
	}
	/*// CP for data with/o removal
	cout << "Values of sonode : " << endl;
	for(int i=0; i<realDime; i++) {
		cout<< sonode[2][i] << " " <<endl; // CP
	}
	for(int i=0; i<colsnode; i++) {
		cout<< csoinn->GetNode(2)->GetSignal()[i] <<" "<<endl; // CP
	}*/
}

void SoiamNode::soiamPrediction(CSOINN *csoinn, double** sonode, double *signal, int rowsnode, int colsnode){
	
	// Calculate distance
	double* m_dist = new double[rowsnode];
	double* so_node = new double[realDime];
	int index[2]={0,0};	// number of Winner index 
	double min_dist = csoinn->INFI;
	double sum;
	for(int i=0;i<rowsnode;i++){
		for(int j=0;j<realDime;j++){
			so_node[j] = sonode[i][j];
		}
		// Get distance
		// m_dist[i] = csoinn->Distance(so_node, signal); // Only when using the full length of data
		sum = 0.0;
		for (int s=0; s<realDime; s++){
			sum += (so_node[s]-signal[s])*(so_node[s]-signal[s]);
		}

		m_dist[i] = sqrt(sum)/(double)(realDime);

		// Find winner index
		if(m_dist[i]<min_dist){
			min_dist = m_dist[i];
			index[1] = i;
		}
	}
	//cout << "Minimum Dist: " << min_dist <<" in "<< index[1] <<endl; // CP
	// Get Value
	double value[Dim];
	int v = 0;
	// cout<< "Predicted :" << endl;	// CP
	for(int c=Key-1;c<colsnode;c+=Key){
		value[v] = csoinn->GetNode(index[1])->GetSignal()[c];
		//cout<< value[v] << endl;	// CP
		v++;
	}
	soiamCMD.pitch	= Pvx = value[(Dim-Key-1)];
	soiamCMD.roll	= Pvy = value[(Dim-Key-1)+1];
	soiamCMD.gaz	= Pvz = value[(Dim-Key-1)+2];
	soiamCMD.yaw	= Pvr = value[(Dim-Key-1)+3];
	sendControlToDrone(soiamCMD);
}

void SoiamNode::sendControlToDrone(ControlCommand predcmd)
{
	// TODO: check converstion (!)
	geometry_msgs::Twist cmdT;
	cmdT.angular.z = predcmd.yaw;
	cmdT.linear.z = predcmd.gaz;
	cmdT.linear.x = predcmd.pitch;
	cmdT.linear.y = predcmd.roll;

	// assume that while actively controlling, the above for will never be equal to zero, so it will never hover.
	cmdT.angular.x = cmdT.angular.y = 0;

	pthread_mutex_lock(&send_CS);
	predict_pub.publish(cmdT);
	pthread_mutex_unlock(&send_CS);
}

void SoiamNode::recFile (double posX, double posY, double vx, double vy, double vz, double vr) 
{
	ofstream rec;
  	rec.open ("/home/originholic/hydro_ws/src/soiam/rec/superhovering.txt", ios::out | ios::app);
  	rec << posX << " " << posY  << " " << vx << " " << vy << " " << vz << " " << vr << endl;
  	rec.close();
}

int SoiamNode::ReadNumbers(string *s, vector<double> &v) {
	istringstream is( *s );
	//string token;
	double n;
	while(is >> n) {	//is >> n or getline(is, token, ' ') ??
		v.push_back( n );
		//v.push_back(atof(token.c_str()));
	}
	return v.size();
}

void SoiamNode::loadTrainingdata(CSOINN *csoinn){

	// Open training data from txt file 
	ifstream rec("/home/originholic/hydro_ws/src/soiam/superhovering.txt");
	string dstr;
	int cols=0, rows=0;
	int g=0;
	vector<double> sdata;

	// Check Training file availability
	if (rec.is_open()) {
		int i=0;
		getline(rec, dstr);
		cols =ReadNumbers(&dstr, sdata);
		//cout << "cols:" << cols << endl; // CP: columns of data
		
		while(getline(rec, dstr)){
				//if ( getline(rec, dstr) == 0 ) break;
				ReadNumbers(&dstr, sdata);
				//cout << dstr << endl;
				i++;
		}			
		rows=i;
		//cout << "rows :" << rows << endl; // CP: rows of data
		rec.close(); // close file after reading

		// Training SOINN
		ROS_INFO("Training Soiam from File ..."); // CP
		// rows * cols is the size of the matrix
		double** sodata = new double*[rows];
		for(int r=0; r<rows; r++){
			sodata[r]=new double[cols];
			for(int c=0; c<cols; c++){
				sodata[r][c] = sdata[g];
				g++;
			}
			csoinn->InputSignal(sodata[r]);
		}

		// Nodes NO. + Check point
		std::cout<<"	Trained with "<< csoinn->GetNodeNum() <<" Nodes"<<endl; // Check point: No. of Nodes
		ROS_INFO("Wait for Real-Time Training ...");
		rowsnode = csoinn->GetNodeNum();
		colsnode = cols;
	}
	else{
		ROS_INFO("Wait for Real-Time Training ...");
	}
}
