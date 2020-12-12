#include "utility.h"
#include "InEKF.h"

using namespace std;
using namespace inekf;

class PoseEstimate{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PoseEstimate();
    void PubPose();
    
    private:
    InEKF* filter;
    bool sysTimeFlag = false;
    bool isFilterInitialed =false;
    ros::Time sysTime;
    ros::Time initTime;
    sensor_msgs::Imu imuGlobal;
    Eigen::Vector3d llaOrigin; // lla refers to latitude longitude and altitide

    // Ros variables
    ros::NodeHandle nh;
    ros::Subscriber gpsFrontSub;
    ros::Subscriber imuSub;
    ros::Subscriber wheelSub;

    ros::Publisher pathPub;
    ros::Publisher gpsFrontPathPub;
    ros::Publisher posePub;
    ros::Publisher resultPub;
    
    tf::StampedTransform odometryTransform;
    tf::TransformBroadcaster tfBroadcaster;
    // Params
    std::string GPS_TOPIC;
    std::string IMU_TOPIC;
    int IS_ZERO_UPDATE;
    int IS_WHEEL_UPDATE;
    int IS_VERBOSE_MODE;
    int IS_GPS_VELOCITY_UPDATE;
    int IS_GPS_POSITION_UPDATE;

    double L_B_0, L_B_1, L_B_2;
    double INIT_ROLL, INIT_PITCH, INIT_YAW;
    double GPS_POS_STD_0, GPS_POS_STD_1, GPS_POS_STD_2;
    double GPS_VEL_STD_0, GPS_VEL_STD_1, GPS_VEL_STD_2;
    double ACC_STD, GYRO_STD, ACC_BIAS_STD, GYRO_BIAS_STD;
    
    // Fuctions
    void LoadParameters();
    void ZeroUpdate(const sensor_msgs::Imu imuMsg);
    bool InititalizeFilter(const Eigen::Vector3d& accBias, Eigen::Vector3d& gyroBias);

    // Callback fuctions
    void GpsFrontHandler(const sensor_msgs::NavSatFixConstPtr& gpsMsg);
    void WheelSpeedHandler(const can_driver::steerWheel_angleConstPtr& wheelMsg);
    void ImuHandler(const sensor_msgs::ImuConstPtr& imuMsg);
};

PoseEstimate::PoseEstimate(){
    
    LoadParameters();
    // ROS
    gpsFrontSub = nh.subscribe<sensor_msgs::NavSatFix>(std::string(GPS_TOPIC), 100, &PoseEstimate::GpsFrontHandler, this);
    imuSub = nh.subscribe<sensor_msgs::Imu>(std::string(IMU_TOPIC), 100, &PoseEstimate::ImuHandler, this);
    wheelSub = nh.subscribe<can_driver::steerWheel_angle>("/Can/SteerWheelAndAngle", 100, &PoseEstimate::WheelSpeedHandler, this);

    pathPub = nh.advertise<nav_msgs::Path>("/imuPath", 100);
    gpsFrontPathPub = nh.advertise<nav_msgs::Path>("/gpsFrontPath", 100);
    posePub = nh.advertise<geometry_msgs::PoseStamped>("/imuPose", 1000);
    resultPub = nh.advertise<sensor_msgs::Imu>("/results", 100);

    odometryTransform.frame_id_  = "/imu_init";
    odometryTransform.child_frame_id_ = "/imu";
}

void PoseEstimate::LoadParameters(){
    // Param
    nh.param<string>("GPS_TOPIC", GPS_TOPIC, "--");
    nh.param<string>("IMU_TOPIC", IMU_TOPIC, "--");

    nh.param<int>("IS_ZERO_UPDATE", IS_ZERO_UPDATE, 0);
    nh.param<int>("IS_WHEEL_UPDATE", IS_WHEEL_UPDATE, 0);

    nh.param<int>("IS_GPS_VELOCITY_UPDATE", IS_GPS_VELOCITY_UPDATE, 1);
    nh.param<int>("IS_GPS_POSITION_UPDATE", IS_GPS_POSITION_UPDATE, 1);

    nh.param<int>("IS_VERBOSE_MODE", IS_VERBOSE_MODE, 0);
    
    nh.param<double>("L_B_0", L_B_0, 0);
    nh.param<double>("L_B_1", L_B_1, 0);   
    nh.param<double>("L_B_2", L_B_2, 0);

    nh.param<double>("INIT_ROLL", INIT_ROLL, 0);
    nh.param<double>("INIT_PITCH", INIT_PITCH, 0);
    nh.param<double>("INIT_YAW", INIT_YAW, 0);

    nh.param<double>("ACC_STD", ACC_STD, 1e-2);
    nh.param<double>("GYRO_STD", GYRO_STD, 1e-4);
    nh.param<double>("ACC_BIAS_STD", ACC_BIAS_STD, 1e-6);
    nh.param<double>("GYRO_BIAS_STD", GYRO_BIAS_STD, 1e-8);

    nh.param<double>("GPS_POS_STD_0", GPS_POS_STD_0, 100);
    nh.param<double>("GPS_POS_STD_1", GPS_POS_STD_1, 100);
    nh.param<double>("GPS_POS_STD_2", GPS_POS_STD_2, 100);

    nh.param<double>("GPS_VEL_STD_0", GPS_VEL_STD_0, 100);
    nh.param<double>("GPS_VEL_STD_1", GPS_VEL_STD_1, 100);
    nh.param<double>("GPS_VEL_STD_2", GPS_VEL_STD_2, 100);
}

bool PoseEstimate::InititalizeFilter(const Eigen::Vector3d& accBias, Eigen::Vector3d& gyroBias){

    ROS_INFO("Initializing the Invariant Extended Kalman Filter");

    Eigen::Vector3d g;
    Eigen::Matrix3d R0;
    Eigen::Vector3d v0, p0, ba0, bg0;
    Eigen::MatrixXd Cov = Eigen::MatrixXd::Zero(15,15);

    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(INIT_ROLL, Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(INIT_PITCH, Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(INIT_YAW, Eigen::Vector3d::UnitZ()));
    
    R0 = yawAngle * pitchAngle * rollAngle;
    v0 << 0, 0, 0;
    p0 << 0, 0, 0;
    g  << 0,0,-9.8015;

    // Set covariance
    Cov.block<3, 3>(0, 0) = 0.0001 * deg_to_rad * 1 * deg_to_rad * Eigen::Matrix3d::Identity();  
    Cov.block<3, 3>(3, 3) = 0.01 * Eigen::Matrix3d::Identity(); // velocity std: 1 m/s
    Cov.block<3, 3>(6, 6) = 0.01 * Eigen::Matrix3d::Identity(); // position std:
    // Acc bias cov
    Cov.block<3, 3>(9, 9) = 0.00004 * Eigen::Matrix3d::Identity();
    // Gyro bias cov
    Cov.block<3, 3>(12, 12) = 0.00004 * Eigen::Matrix3d::Identity();

    //Initialize state
    RobotState initState;
    initState.setRotation(R0);
    initState.setVelocity(v0);
    initState.setPosition(p0);
    initState.setGyroscopeBias(gyroBias);
    initState.setAccelerometerBias(accBias);
    initState.setP(Cov);
    //Initialize state covariance
    NoiseParams noiseParams;
    noiseParams.setGyroscopeNoise(Eigen::Vector3d(GYRO_STD,GYRO_STD,GYRO_STD));
    noiseParams.setAccelerometerNoise(Eigen::Vector3d(ACC_STD,ACC_STD,ACC_STD));
    noiseParams.setGyroscopeBiasNoise(Eigen::Vector3d(GYRO_BIAS_STD,GYRO_BIAS_STD,GYRO_BIAS_STD));
    noiseParams.setAccelerometerBiasNoise(Eigen::Vector3d(ACC_BIAS_STD,ACC_BIAS_STD,ACC_BIAS_STD));
    //Initialize filter
    filter = new InEKF(initState, noiseParams);
    return true;
}

void PoseEstimate::ZeroUpdate(const sensor_msgs::Imu imuMsg){
     
    // Correct Acclerator Bias
    Eigen::MatrixXd H2;
    Eigen::Vector3d Z2;
    Eigen::MatrixXd N2;
    Eigen::Matrix3d Nba = Eigen::Matrix3d::Identity();
    Eigen::Vector3d g_(0,0,-9.8015);

    // Fill out H2
    // H2.conservativeResize(3, 15);
    // H2.block(0, 0, 3, 15) = Eigen::MatrixXd::Zero(3, 15);
    // H2.block(0, 0, 3, 3) = - R.transpose() * skew(g_);
    // H2.block(0, 12, 3, 3) =  Eigen::Matrix3d::Identity();
    // // Fill out Z2
    // Z2 <<   -imuMsg->linear_acceleration.y,
    //         imuMsg->linear_acceleration.x,
    //         imuMsg->linear_acceleration.z;
    // Z2 = Z2 - filter->getState().getAccelerometerBias() + R.transpose() * g_;  

    // Fill out H2
    H2.conservativeResize(3, 15);
    H2.block(0, 0, 3, 15) = Eigen::MatrixXd::Zero(3, 15);
    H2.block(0, 12, 3, 3) =  Eigen::Matrix3d::Identity();

    // Fill out Z2
    Z2 <<   imuMsg.linear_acceleration.x,
            imuMsg.linear_acceleration.y,
            imuMsg.linear_acceleration.z;
    Z2 = Z2 - filter->getState().getAccelerometerBias() + g_;  

    // Fill out N2
    int startIndex = N2.rows();
    N2.conservativeResize(startIndex + 3, startIndex +3);
    N2.block(startIndex,startIndex,3,3) = 0.00001 * Nba;

    filter->RightCorrect(H2,Z2,N2);


    // Correct Gyroscope Bias
    Z2 <<   imuMsg.angular_velocity.x,
            imuMsg.angular_velocity.y,
            imuMsg.angular_velocity.z;
    Z2 = Z2 - filter->getState().getGyroscopeBias();

    // Fill out H2
    H2.conservativeResize(3, 15);
    H2.block(0, 0, 3, 15) = Eigen::MatrixXd::Zero(3, 15);
    H2.block(0, 9, 3, 3) =  Eigen::Matrix3d::Identity();

    filter->RightCorrect(H2,Z2,N2);

}

void PoseEstimate::PubPose(){

    if (IS_VERBOSE_MODE){
        // Stream out state
        // cout << "rotation" << endl;
        // cout << filter->getState().getRotation() << endl;

        // cout << "position:" << endl;
        // cout << filter->getState().getPosition() << endl;

        // cout << "velocity:" << endl;
        // cout << filter->getState().getVelocity() << endl;

        // cout << "theta:" << endl;
        // cout << "Gyro and Acc bias are:" << endl; 
        // cout << filter->getState().getAccelerometerBias() << "|||" << filter->getState().getGyroscopeBias()<< endl << endl;
        // cout << "P is" << endl;
        // cout << filter->getState().getP() << endl << endl;

        // Eigen::Vector3d vb = filter->getState().getRotation().transpose() * filter->getState().getVelocity();
        // double sideangle = atan2(vb(1), vb(0)) * 180.0 / 3.1415920;
        // cout << sideangle << endl;
    }
    //Publish sideslip angel
    Eigen::Vector3d vb = filter->getState().getRotation().transpose() * filter->getState().getVelocity();
    double sideangle = atan2(vb(1), vb(0)) * 180.0 / 3.1415920;
    sensor_msgs::Imu msg;
    msg.header.stamp = sysTime; 
    msg.angular_velocity.z = sideangle;
    resultPub.publish(msg);

    //Publish tf and trajactory
    odometryTransform.frame_id_  = "/imu_init";
    odometryTransform.child_frame_id_ = "/imu";
    odometryTransform.stamp_ = sysTime; 
    Eigen::Quaterniond q(filter->getState().getRotation());
    Eigen::Vector3d p(filter->getState().getPosition());
    odometryTransform.setOrigin(tf::Vector3(p(0), p(1), p(2)));
    odometryTransform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    tfBroadcaster.sendTransform(odometryTransform);

    odometryTransform.frame_id_  = "/imu_init";
    odometryTransform.child_frame_id_ = "/velodyne";
    tfBroadcaster.sendTransform(odometryTransform);
    // Publish trajactory
    static nav_msgs::Path pathMsg;
    geometry_msgs::PoseStamped pose;

    pose.header.frame_id = "imu_init";
    pose.header.stamp = sysTime;

    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    pose.pose.position.x = p(0);
    pose.pose.position.y = p(1);
    pose.pose.position.z = p(2);
    
    posePub.publish(pose);

    pathMsg.header.stamp = sysTime; 
	pathMsg.header.frame_id="imu_init"; 
    pathMsg.poses.push_back(pose);
    pathPub.publish(pathMsg);
}

void PoseEstimate::ImuHandler(const sensor_msgs::ImuConstPtr& imuMsg){
    static int count = 0;
    static Eigen::Vector3d gyroBiasSum(0,0,0), accBiasSum(0,0,0);
    if(!isFilterInitialed){
        gyroBiasSum = gyroBiasSum + Eigen::Vector3d(imuMsg->angular_velocity.x,
                                                    imuMsg->angular_velocity.y, 
                                                    imuMsg->angular_velocity.z);
        accBiasSum = accBiasSum + Eigen::Vector3d(imuMsg->linear_acceleration.x,
                                                  imuMsg->linear_acceleration.y, 
                                                  imuMsg->linear_acceleration.z - 9.8015);                           
        count++;
        if(count == 5000){
        gyroBiasSum = gyroBiasSum / count;
        accBiasSum = accBiasSum / count;
        isFilterInitialed  = InititalizeFilter(accBiasSum, gyroBiasSum);
        sysTime =  imuMsg->header.stamp;
        initTime = sysTime;
        sysTimeFlag = true;
        return;
        }
    }
    // If filter not initialized, return
    if(!isFilterInitialed) return;
    // Propegate KF using IMU measurement 
    double dt = imuMsg->header.stamp.toSec() - sysTime.toSec();
    sysTime = imuMsg->header.stamp;// Update system time for next step
    imuGlobal = *imuMsg;// update imu msg for global use
    //Load Imu data
    static Eigen::Matrix<double,6,1> imuMeasurement = Eigen::Matrix<double,6,1>::Zero();
    imuMeasurement << imuMsg->angular_velocity.x,    imuMsg->angular_velocity.y,    imuMsg->angular_velocity.z,
                      imuMsg->linear_acceleration.x, imuMsg->linear_acceleration.y, imuMsg->linear_acceleration.z;
    //Propagate filter 
    filter->LeftPropagate(imuMeasurement, dt);
    //Publish pose
    PubPose();
}

void PoseEstimate::GpsFrontHandler(const sensor_msgs::NavSatFixConstPtr& gpsMsg){

    static nav_msgs::Path pathMsg;
    static geometry_msgs::PoseStamped pose;
    int gpsCovarianceType = 0;  // 131 for rtk, 67 for float, 3 for single solution
    
    if (!isFilterInitialed){
        llaOrigin << gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude;
        return;
    }
    Eigen::Vector3d lla(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude);
    Eigen::Vector3d posENU;
    ConvertLLAToENU(llaOrigin, lla, &posENU);

    // Position Correct
    if (IS_GPS_POSITION_UPDATE){ 

        Eigen::MatrixXd H;
        Eigen::VectorXd Z;
        Eigen::MatrixXd N;
        posENU = posENU - filter->getState().getRotation() * Eigen::Vector3d(0, 0 ,0);
        gpsCovarianceType = int(gpsMsg->position_covariance_type);

        H.conservativeResize(3, 15);
        H.block(0, 0, 3, 15) = Eigen::MatrixXd::Zero(3, 15);
        H.block(0, 6, 3, 3) =  Eigen::Matrix3d::Identity();
    
        Z.conservativeResize(3, Eigen::NoChange);
        Z.segment(0,3) = filter->getState().getRotation().transpose() * (posENU - filter->getState().getPosition());

        Eigen::Vector3d gps_pos_noise(GPS_POS_STD_0*GPS_POS_STD_0, GPS_POS_STD_1*GPS_POS_STD_1, GPS_POS_STD_2*GPS_POS_STD_2);
        Eigen::MatrixXd Ngps = filter->getState().getRotation().transpose() 
                             * gps_pos_noise.asDiagonal() 
                             * filter->getState().getRotation(); 
        Eigen::MatrixXd N_PP7 = filter->getState().getRotation().transpose()
                              * Eigen::Map<const Eigen::Matrix3d>(gpsMsg->position_covariance.data())
                              * filter->getState().getRotation();
        // filter->LeftCorrect(H,Z,N_PP7);
        if (gpsCovarianceType == 131){
            N = Ngps;
            filter->LeftCorrect(H,Z,N);
        } 
        else if (gpsCovarianceType == 67){
            N =  10* Ngps;
            filter->LeftCorrect(H,Z,N);
        }
        else if (gpsCovarianceType == 1){
            N =  100* Ngps;
            filter->LeftCorrect(H,Z,N);
        }
    }

    // Velocity Correct
    if (IS_GPS_VELOCITY_UPDATE ) {   //&& ((sysTime.toSec() - initTime.toSec()) < 100) 

        Eigen::Vector3d velENU;  // in NED frame
        Eigen::Vector3d angularVelocity;
        angularVelocity << imuGlobal.angular_velocity.x, imuGlobal.angular_velocity.y, imuGlobal.angular_velocity.z;
        velENU << gpsMsg->position_covariance[5], gpsMsg->position_covariance[4], -gpsMsg->position_covariance[6];
        velENU -= filter->getState().getRotation() * angularVelocity.cross(Eigen::Vector3d(0, 0 ,0)); 
        Eigen::MatrixXd H;
        Eigen::VectorXd Z;
        Eigen::MatrixXd N;
        
        H.conservativeResize(3, 15);
        H.block(0, 0, 3, 15) = Eigen::MatrixXd::Zero(3, 15);
        H.block(0, 3, 3, 3) =  Eigen::Matrix3d::Identity();
        
        Z.conservativeResize(3, Eigen::NoChange);
        Z.segment(0,3) =  filter->getState().getRotation().transpose() * (velENU - filter->getState().getVelocity());

        // Fill out N
        Eigen::Vector3d gps_vel_noise(GPS_VEL_STD_0*GPS_VEL_STD_0, GPS_VEL_STD_1*GPS_VEL_STD_1, GPS_VEL_STD_2*GPS_VEL_STD_2);
        N = filter->getState().getRotation().transpose() 
          * gps_vel_noise.asDiagonal() 
          * filter->getState().getRotation(); 
        // double gpsVelStd = gpsMsg->position_covariance[7];
        filter->LeftCorrect(H,Z,N);
    }

    if (0){
        filter->LeftToRight();
        Eigen::MatrixXd H;
        Eigen::VectorXd Z;
        Eigen::MatrixXd N;
        
        Eigen::Vector3d temp = filter->getState().getRotation().transpose() * filter->getState().getVelocity();
        Z.conservativeResize(2, Eigen::NoChange);
        Z = -temp.segment(1,2);

        H.conservativeResize(2,15);
        H.block(0, 0, 2, 15) = Eigen::MatrixXd::Zero(2, 15);
        H.block(0, 3, 2, 3) = filter->getState().getRotation().transpose().block(1, 0 ,2, 3);
        
        N.conservativeResize(2, 2);
        N = 10* Eigen::Matrix2d::Identity();
        ROS_INFO("zero update");
        filter->RightCorrect(H, Z, N);

        filter->RightToLeft();
    }
        
    // Pub GPS path for compare
    pose.header.frame_id = "gps_front";
    pose.header.stamp = ros::Time::now();

    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 0;

    pose.pose.position.x = posENU(0);
    pose.pose.position.y = posENU(1);
    pose.pose.position.z = posENU(2);

    pathMsg.header.stamp=ros::Time::now(); 
	pathMsg.header.frame_id="imu_init"; 

    pathMsg.poses.push_back(pose);
    gpsFrontPathPub.publish(pathMsg);
   
}   

void PoseEstimate::WheelSpeedHandler(const can_driver::steerWheel_angleConstPtr& wheelMsg){
    if (!isFilterInitialed) return;
    double rl, rr;
    rl = wheelMsg->wheelSpeed_RL / 295.0 ;
    rr = wheelMsg->wheelSpeed_RR / 295.0 ;
    if(rl <= 0.005 && rr <= 0.005 && sysTimeFlag){
        // ZeroUpdate(imuGlobal);
        return;
    }

    if (IS_ZERO_UPDATE && abs(rl-rr) <0.05 && (rl > 3)){

        Eigen::MatrixXd H;
        Eigen::VectorXd Z;
        Eigen::MatrixXd N;
        
        Eigen::Vector3d temp = filter->getState().getRotation().transpose() * filter->getState().getVelocity();
        Z.conservativeResize(2, Eigen::NoChange);
        Z = -temp.segment(1,2);

        H.conservativeResize(2,15);
        H.block(0, 0, 2, 15) = Eigen::MatrixXd::Zero(2, 15);
        H.block(0, 3, 2, 3) = filter->getState().getRotation().transpose().block(1, 0 ,2, 3);
        
        N.conservativeResize(2, 2);
        N = 10 * Eigen::Matrix2d::Identity();
        ROS_INFO("zero update");
        filter->LeftToRight();
        filter->RightCorrect(H, Z, N);
        filter->RightToLeft();
    }

    if (IS_WHEEL_UPDATE && abs(rl-rr) <0.05 && rl > 5){
        filter->LeftToRight();
        Eigen::MatrixXd H;
        Eigen::VectorXd Z;
        Eigen::MatrixXd N;
            
        Eigen::Vector3d temp = filter->getState().getRotation().transpose() * filter->getState().getVelocity();
        Z.conservativeResize(3, Eigen::NoChange);
        Z = -temp;
        Z(0) -= 0.5 * (rl+ rr);

        H.conservativeResize(3,15);
        H.block(0, 0, 3, 15) = Eigen::MatrixXd::Zero(3, 15);
        H.block(0, 3, 3, 3) = filter->getState().getRotation().transpose().block(0, 0 ,3, 3);
        
        N.conservativeResize(3, 3);
        N = 1 * Eigen::Matrix3d::Identity();
        
  
        // filter->RightCorrect(H, Z, N);
    }


} 

int main(int argc, char** argv){
    ros::init(argc, argv, "sensor_fusion");
    ROS_INFO("start sensor fusion ");

    PoseEstimate estimater;
    
    ros::Rate rate(500);
    while (ros::ok()){   
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}


