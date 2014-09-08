#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/tfMessage.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "ardrone_autonomy/Navdata.h"
#include "tum_ardrone/filter_state.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "cvd/thread.h"

#include "SOINN.h"

#define Key 3				// Key length(fold) of data
#define Dim 6				// X, Y, cmd vx-vy-vz-vr
#define CMD 4				// cmd: vx, vy, vz, vr
#define	realDime 4			// (Key-1)*(Dim-CMD)

struct LearningSet
{
	inline LearningSet() {xpos = ypos = xlin = ylin = zlin = zang = 0;}
	inline LearningSet(double xpos, double ypos, double xlin, double ylin, double zlin, double zang)
	{
		this->xpos = xpos;
		this->ypos = ypos;		
		this->xlin = xlin;
		this->ylin = ylin;
		this->zlin = zlin;
		this->zang = zang;
	}
	double xpos, ypos, xlin, ylin, zlin, zang;
	//double zpos, rpos;
};

struct ControlCommand
{
	inline ControlCommand() {roll = pitch = yaw = gaz = 0;}
	inline ControlCommand(double roll, double pitch, double yaw, double gaz)
	{
		this->roll = roll;
		this->pitch = pitch;
		this->yaw = yaw;
		this->gaz = gaz;
	}
	double yaw, roll, pitch, gaz;
};

class SoiamNode
{
private:

	ros::NodeHandle nh_;

	// subscribe
	ros::Subscriber pose_sub;
	ros::Subscriber vel_sub; // to co-read contro commands sent from other thread
	ros::Subscriber com_sub;
	ros::Subscriber isLearning_sub;
	ros::Subscriber isPredict_sub;
	ros::Time lastNavStamp;

	// publish
	ros::Publisher predict_pub;

	// parameters
	// every [publishFreq]ms, the node publishes the drones predicted position [predTime]ms into the future.
	// this pose can then be used to steer the drone. obviously, the larger [predTime], the worse the estimate.
	// this pose is published on /tf, and simultaneously further info is published on /ardrone/predictedPose
	ros::Duration predTime;
	std::string pose_channel;
	std::string control_channel;
	std::string common_channel;
	std::string predict_channel;
	std::string learning_toggle;
	std::string predicting_toggle;

	int publishFreq;
	int minPublishFreq;
		
	LearningSet learningSet;

	static pthread_mutex_t send_CS;

	inline void recFile(double posX, double posY, double vx, double vy, double vz, double vr);

public:
	//std::string packagePath;
	CSOINN *csoinn;
	CSOINN *zsoinn;

	SoiamNode();
	~SoiamNode();

	// ROS message callbacks
	void velCb(const geometry_msgs::TwistConstPtr velPtr);
	void poseCb(const tum_ardrone::filter_stateConstPtr statePtr);
	void comCb(const std_msgs::StringConstPtr str);
	void learnCb(const std_msgs::Bool::ConstPtr learn);
	void predCb(const std_msgs::Bool::ConstPtr pred);
	//void dynConfCb(tum_ardrone::StateestimationParamsConfig &config, uint32_t level);
	void sendControlToDrone(ControlCommand predcmd);

	// main soiam loop
	void Loop();
	// am functions
	void nodetoSoinn(CSOINN *csoinn, double *signal);
	void soiamFilter(CSOINN *csoinn, int rowsnode, int colsnode);
	void soiamPrediction(CSOINN *csoinn, double** sonode, double *signal, int rowsnode, int colsnode);
	int ReadNumbers(string *s, vector<double> &v);
	void loadTrainingdata(CSOINN *csoinn);

	// internal
	ControlCommand soiamCMD;
	bool isLearning, isPredicting;
	double rowsnode, colsnode;
	double Xbuf[Key], cXbuf[Key], Ybuf[Key], cYbuf[Key];
	double pXbuf[Key], pYbuf[Key];
	double Pvx, Pvy, Pvz, Pvr;
	double** filteredNode;
};
