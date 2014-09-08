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
	randomCMD.gaz = randomCMD.pitch = randomCMD.roll = randomCMD.yaw = 0;
	for(int b=0;b<JIT_m+JIT_p;b++){
		Xbuf[b] = Ybuf[b] = 0;
	}
	for(int b=0;b<JIT_n+JIT_p;b++){
		cXbuf[b] = cYbuf[b] = 0;
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
	Xsoinn = new CSOINN(Dim,40000,850,1);
	Ysoinn = new CSOINN(Dim,40000,850,1);
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

	/*
	const char learnItem[Dim*Key][6] = 
	{
		{"X1"},{"X2"},{"X3"},{"Y1"},{"Y2"},{"Y3"},
		{"cmdX1"},{"cmdX2"},{"cmdX3"},
		{"cmdY1"},{"cmdY2"},{"cmdY3"},
		{"cmdZ1"},{"cmdZ2"},{"cmdZ3"},
		{"cmdR1"},{"cmdR2"},{"cmdR3"}
	};
	*/

	std::cout << "Add new Node to SOINN..." << endl; // CP
	// add one row a time when this is called
	csoinn->InputSignal(signal);

	// Nodes NO. + Check point
	std::cout<<"Finished Training with "<< csoinn->GetNodeNum() <<" Nodes"<<endl; // CP: No. of Nodes
	rowsnode = csoinn->GetNodeNum();
	//colsnode = Dim*Key;

	// Loop of CP
	/*
	cout << "Values of sonode : " << endl;
	for(int i=0; i<Dim*Key; i++) {
		cout<< learnItem[i] << ": " << signal[i] << " " <<endl; // CP
		//cout<< csoinn->GetNode(2)->GetSignal()[i] <<" "<<endl; // CP
	}
	*/
}

void SoiamNode::Loop()
{
	ros::Time last = ros::Time::now();
	ros::Time lastStateUpdate = ros::Time::now();
	Xmaxpos = loadTrainingdata(1, Xsoinn);
	Xrowsnode =rowsnode;
	Ymaxpos = loadTrainingdata(2, Ysoinn);
	Yrowsnode =rowsnode;

	while (nh_.ok())
	{
		// -------------- 1. spin for 50ms, do main controlling part here. ---------------
		while((ros::Time::now() - last) < ros::Duration(minPublishFreq / 1000.0))
			ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(minPublishFreq / 1000.0 - (ros::Time::now() - last).toSec()));
		last = ros::Time::now();

		// -------------- 2. train soinn. (maybe). do something good. (hope). ------------
	
		
		//trainning
		if(isLearning)
		{


			//random input
			randomCMD.pitch = RandomInput(-1,1);
			randomCMD.roll = RandomInput(-1,1);
			randomCMD.gaz = 0.0;
			randomCMD.yaw = 0.0;
			sendControlToDrone(randomCMD);
			

			//buf[0]:pos[t-m+1]~buf[JIT_m+JIT_p-2]:pos[t+p-1]
			for(int b=0;b<JIT_m+JIT_p-1;b++){
				Xbuf[b] = Xbuf[b+1];
				Ybuf[b] = Ybuf[b+1];
			}
			//cbuf[0]:cmd[t-n+1]~cbuf[JIT_m+JIT_p-2]:cmd[t+p-1]
				for(int b=0;b<JIT_n+JIT_p-1;b++){
				cXbuf[b] = cXbuf[b+1];
				cYbuf[b] = cYbuf[b+1];
			}
			//buf[JIT_m+JIT_p-1]:pos[t+p] , cbuf[JIT_n+JIT_p-1]:cmd[t+p]
			Xbuf[JIT_m+JIT_p-1] = learningSet.xpos;
			cXbuf[JIT_n+JIT_p-1] = randomCMD.pitch; //learningSet.xlin;	//randomCMD.pitch
			Ybuf[JIT_m+JIT_p-1] = learningSet.ypos;
			cYbuf[JIT_n+JIT_p-1] = randomCMD.roll; //learningSet.ylin;	//randomCMD.roll

			//Xonsignal
			double Xonsignal[Dim]; // Remove command: records with -CMD
			for(int f=0;f<JIT_m+JIT_p;f++){
				Xonsignal[f] = Xbuf[f];			// X[t-m+1]~x[t+p]
			}
			for(int f=0;f<JIT_n+cmdKey;f++){
				Xonsignal[f+JIT_m+JIT_p] = cXbuf[f];	// cmdX[t-n+1]~cmdX[t+1]
			}
		
			//Yonsignal
			double Yonsignal[Dim]; // Remove command: records with -CMD
			for(int f=0;f<JIT_m+JIT_p;f++){
				Yonsignal[f] = Ybuf[f];			// Y[t-m+1]~Y[t+p]
			}
			for(int f=0;f<JIT_n+cmdKey;f++){
				Yonsignal[f+JIT_m+JIT_p] = cYbuf[f];	// cmdY[t-n+1]~cmdY[t+1]
			}

			//record
			recFile (1, Xonsignal);//record Xonsignal
			recFile (2, Yonsignal);//record Yonsignal

			//normalization
			for(int f=0;f<JIT_m+JIT_p;f++){
				Xonsignal[f] = Xonsignal[f]/Xmaxpos;			// X[t-m+1]~x[t+p]
			}

			for(int f=0;f<JIT_m+JIT_p;f++){
				Yonsignal[f] = Yonsignal[f]/Ymaxpos;			// Y[t-m+1]~Y[t+p]
			}
			
			// Add node to Xsoinn
			nodetoSoinn(Xsoinn, Xonsignal);
			Xrowsnode =rowsnode;
			// Add node to Ysoinn
			nodetoSoinn(Ysoinn, Yonsignal);
			Yrowsnode =rowsnode;

			
		}

		//predicting
		if(isPredicting)
		{

			//record Position & SoiamCommand
			recFile2(learningSet.xpos, learningSet.ypos, Pvx, Pvy);
			//record Position & Command
			recFile3(learningSet.xpos,learningSet.ypos,learningSet.xlin,learningSet.ylin);

			//pos[t-m+1]~pos[t]
			for(int b=0;b<JIT_m-1;b++){
				pXbuf[b] = pXbuf[b+1];
				pYbuf[b] = pYbuf[b+1];
			}

			pXbuf[JIT_m-1] = learningSet.xpos;
			pYbuf[JIT_m-1] = learningSet.ypos;

			//reference
			//r[t+1]~r[t+p]
			for(int b=0;b<JIT_p-1;b++){
				rXbuf[b] = rXbuf[b+1];
				rYbuf[b] = rYbuf[b+1];
			}
			rXbuf[JIT_p-1]=0.0;
			rYbuf[JIT_p-1]=0.0;

			//cmd[t-n+1]~cmd[t]
			for(int b=0;b<JIT_n-1;b++){
				pcXbuf[b] = pcXbuf[b+1];
				pcYbuf[b] = pcYbuf[b+1];
			}

			pcXbuf[JIT_n-1] = soiamCMD.pitch; //learningSet.xlin;	//soiamCMD.pitch
			pcYbuf[JIT_n-1] = soiamCMD.roll; //learningSet.ylin;	//siamCMD.roll

			// Popout the signal set to recall
			//kNN
			int k = 5;
			//X soiam
			double PXsignal[Dim-1];
			for(int f=0;f<JIT_m;f++){
				PXsignal[f] = pXbuf[f]/Xmaxpos;			// X[t-m+1]~X[t]
			}
			for(int f=0;f<JIT_p;f++){
				PXsignal[f+JIT_m] = rXbuf[f]/Xmaxpos;	// rX[t+1]~rX[t+p]
			}
			for(int f=0;f<JIT_n;f++){
				PXsignal[f+JIT_m+JIT_p] = pcXbuf[f];		// cmdX[t-n+1]~cmd[t]
			}
			
			// TODO:recall from SOIAM
			soiamCMD.pitch = Pvx = soiamPrediction(Xsoinn, PXsignal, Xrowsnode, k);
			
			//Y soiam
			double PYsignal[Dim-1];
			for(int f=0;f<JIT_m;f++){
				PYsignal[f] = pYbuf[f]/Ymaxpos;			// Y[t-m+1]~Y[t]
			}
			for(int f=0;f<JIT_p;f++){
				PYsignal[f+JIT_m] = rYbuf[f]/Ymaxpos;		// rY[t+1]~rY[t+p]
			}
			for(int f=0;f<JIT_n;f++){
				PYsignal[f+JIT_m+JIT_p] = pcYbuf[f];			// cmdY[t-n+1]~cmd[t]
			}
			
			// TODO:recall from SOIAM
			soiamCMD.roll = Pvy = soiamPrediction(Ysoinn, PYsignal, Yrowsnode, k);

			soiamCMD.gaz = Pvz = 0.0;
			soiamCMD.yaw = Pvr = 0.0;
			sendControlToDrone(soiamCMD);
	
			std::cout << "pitch : " << Pvx << ",    roll : " << Pvy << endl;
			//std::cout << "CMDpitch : " << soiamCMD.pitch<< ",    CMDroll : " << soiamCMD.roll << endl; // CP
			//std::cout << endl;


		}

		
		//cout<< isLearning <<endl; // CP
		// -------------- 2. update time. ---------------
		if((ros::Time::now() - lastStateUpdate) > ros::Duration(0.4))
		{
			lastStateUpdate = ros::Time::now();
		}



	}
}


double SoiamNode::soiamPrediction(CSOINN *csoinn, double *signal, int rowsnode, int k){
	
	// Calculate distance
	double* m_dist = new double[rowsnode];
	double* so_node = new double[Dim-1];
	double sum;
	std::vector<std::pair<double,int> > data;
	std::vector<std::pair<double,int> >::iterator it;
	

	for(int i=0;i<rowsnode;i++){
		for(int j=0;j<Dim-1;j++){
			so_node[j] = csoinn->GetNode(i)->GetSignal()[j];//pos[t-m+1]~pos[t+p],cmd[t-n+1]~cmd[t]
		}
		// Get distance
		// m_dist[i] = csoinn->Distance(so_node, signal); // Only when using the full length of data
		sum = 0.0;
		for (int s=0; s<Dim-1; s++){
			sum += (so_node[s]-signal[s])*(so_node[s]-signal[s]);
		}

		m_dist[i] = sqrt(sum)/(double)(Dim-1);
		data.push_back(std::pair<double,int>(m_dist[i],i));

	}
	//sort with index
	std::sort(data.begin(),data.end());
	it=data.begin();
	//kNN
	double value = 0.0;
	double predcmd = 0.0;
	for(int i=0;i<k;i++){
		value += csoinn->GetNode(it->second)->GetSignal()[Dim-1];//cmd[t+1]
		it++;
	}
	predcmd = value/k;

	return predcmd;

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

void SoiamNode::recFile (int a, double *signal) 
{
	ofstream rec;
	if(a==1){
  	rec.open ("/home/enomoto/catkin_ws/src/soiam/rec/Xonsignal.txt", ios::out | ios::app);
	}else if(a==2){
  	rec.open ("/home/enomoto/catkin_ws/src/soiam/rec/Yonsignal.txt", ios::out | ios::app);
	}
	for(int i=0;i<Dim;i++){
  	rec << signal[i] << " " << flush ;
	}
  	rec << endl;
  	rec.close();
}

void SoiamNode::recFile2 (double posX, double posY, double Pvx, double Pvy) 
{
	ofstream rec;
  	rec.open ("/home/enomoto/catkin_ws/src/soiam/rec/Position_cmd(Soiam).txt", ios::out | ios::app);
  	rec << posX << " " << posY  << " " << Pvx << " " << Pvy << endl;
  	rec.close();
}

void SoiamNode::recFile3 (double posX, double posY, double vx, double vy) 
{
	ofstream rec;
  	rec.open ("/home/enomoto/catkin_ws/src/soiam/rec/Position_cmd.txt", ios::out | ios::app);
  	rec << posX << " " << posY  << " " << Pvx << " " << Pvy << endl;
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

double SoiamNode::loadTrainingdata(int a, CSOINN *csoinn){
	// Open training data from txt file 
	ifstream rec;
	if(a==1){
  	rec.open ("/home/enomoto/catkin_ws/src/soiam/Xonsignal.txt", ios::out | ios::app);
	ROS_INFO("Training X Soiam from File ..."); // CP
	}else if(a==2){
  	rec.open ("/home/enomoto/catkin_ws/src/soiam/Yonsignal.txt", ios::out | ios::app);
	ROS_INFO("Training Y Soiam from File ..."); // CP
	}
	string dstr;
	int cols=0, rows=0;
	int g=0;
	double maxpos=0.0;
	vector<double> sdata;

	// Check Training file availability
	if (rec.is_open()) {
		int i=1;
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

		//MaxPosition
		int s=0;
		for(int i=0;i<cols*rows;i++){
			if(s<JIT_m+JIT_p){
				if(fabs(sdata[i])>maxpos){
					maxpos=fabs(sdata[i]);
				}
			}
			s++;
			if(s%cols==0){
				s=0;
			}
		}
				

		// Training SOINN
		// rows * cols is the size of the matrix
		double** sodata = new double*[rows];
		for(int r=0; r<rows; r++){
			sodata[r]=new double[cols];
			for(int c=0; c<cols; c++){
				//position
				if(c<JIT_m+JIT_p){
					sodata[r][c] = sdata[g]/maxpos;
				//command
				}else{
					sodata[r][c] = sdata[g];
				}
				g++;
			}
			csoinn->InputSignal(sodata[r]);
		}

		// Nodes NO. + Check point
		std::cout<<"	Trained with "<< csoinn->GetNodeNum() <<" Nodes"<<endl; // Check point: No. of Nodes
		//ROS_INFO("Wait for Real-Time Training ...");
		rowsnode = csoinn->GetNodeNum();
		colsnode = cols;
	}
	else{
		ROS_INFO("Wait for Real-Time Training ...");
	}

	if(maxpos==0.0){
		std::cout<<"	MAX Position =  1.0"<< endl;
		return 1.0;
	}else{
		std::cout<<"	MAX Position =  "<< maxpos <<endl;
		return maxpos;
	}

}


double SoiamNode::RandomInput(int min, int max){
	static int flag;
	if(flag==0){
		srand((unsigned int)time(NULL));
		flag=1;
	}
	return min+(max-min)*(double)rand()/RAND_MAX;
}

