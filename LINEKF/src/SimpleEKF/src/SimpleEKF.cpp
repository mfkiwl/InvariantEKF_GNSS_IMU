
#include "EKF.h"
#include "common.h"


using namespace simple_ekf;
using namespace std;

class SimpleEKF{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SimpleEKF();
    void PubState();

    private:
    Ekf* filter_;
    ros::Time sysTime;
    ros::Time initTime;
    bool sysTimeFlag = false;
    bool isFilterInitialed =false;
    Eigen::Vector3d llaOrigin;
    sensor_msgs::Imu imuGlobal;

    // Parameters 
    int IS_VERBOSE_MODE;
    int IS_GPS_VELOCITY_UPDATE;
    int IS_GPS_POSITION_UPDATE;
    int IS_ZERO_UPDATE;

    double L_B_0, L_B_1, L_B_2;
    double INIT_ROLL, INIT_PITCH, INIT_YAW;
    double ACC_STD_0, ACC_STD_1, ACC_STD_2;
    double ACC_BIAS_STD_0, ACC_BIAS_STD_1, ACC_BIAS_STD_2;
    double GYRO_STD_0, GYRO_STD_1, GYRO_STD_2;
    double GYRO_BIAS_STD_0, GYRO_BIAS_STD_1, GYRO_BIAS_STD_2;
    double GPS_POS_STD_0, GPS_POS_STD_1, GPS_POS_STD_2;
    double GPS_VEL_STD_0, GPS_VEL_STD_1, GPS_VEL_STD_2;

    // ROS Variables
    ros::NodeHandle nh;
    ros::Subscriber gpsFrontSub;
    ros::Subscriber gpsRearSub;
    ros::Subscriber imuSub;
    ros::Subscriber lidarSub;
    ros::Subscriber wheelSub;
    ros::Subscriber novatelSub;

    ros::Publisher resultPub;
    ros::Publisher gpsPub;
    ros::Publisher pathPub;
    ros::Publisher gpsRearPathPub;
    ros::Publisher gpsFrontPathPub;
    
    tf::StampedTransform odometryTransform;
    tf::TransformBroadcaster tfBroadcaster;
    // Functions 
    void LoadParameters();
    bool InititalizeFilter(const Eigen::Vector3d& accBias, Eigen::Vector3d& gyroBias);
    // Callback fuctions
    void GpsFrontHandler(const sensor_msgs::NavSatFixConstPtr& gpsMsg);
    void GpsRearHandler(const sensor_msgs::NavSatFixConstPtr& gpsMsg);
    void ImuHandler(const sensor_msgs::ImuConstPtr& imuMsg);
    void WheelSpeedHandler(const can_driver::steerWheel_angleConstPtr& wheelMsg);
    void NovatelPvxHandler(const novatel_gps_msgs::InspvaxConstPtr& pvxMsg);
};

SimpleEKF::SimpleEKF(){
    gpsFrontSub = nh.subscribe<sensor_msgs::NavSatFix>("/NeoM8p/Gps_rear", 10, &SimpleEKF::GpsFrontHandler, this);
    gpsRearSub = nh.subscribe<sensor_msgs::NavSatFix>("/NeoM8p/Gps_rear", 10, &SimpleEKF::GpsRearHandler, this);
    imuSub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 100, &SimpleEKF::ImuHandler, this);
    wheelSub = nh.subscribe<can_driver::steerWheel_angle>("/Can/SteerWheelAndAngle", 100, &SimpleEKF::WheelSpeedHandler, this);
    novatelSub = nh.subscribe<novatel_gps_msgs::Inspvax>("/inspvax", 100, &SimpleEKF::NovatelPvxHandler, this);

    resultPub = nh.advertise<sensor_msgs::Imu>("/ins/results", 100);
    pathPub = nh.advertise<nav_msgs::Path>("/imuPath", 100);
    gpsPub = nh.advertise<sensor_msgs::NavSatFix>("/gps",100);
    gpsFrontPathPub = nh.advertise<nav_msgs::Path>("/gpsFrontPath", 100);
    gpsRearPathPub = nh.advertise<nav_msgs::Path>("/gpsRearPath", 100);
    LoadParameters();
}

void SimpleEKF::LoadParameters(){
    //ROS parameter
    // nh.param<int>("IS_ZERO_INITIAL", IS_ZERO_INITIAL, 0);
    nh.param<int>("IS_ZERO_UPDATE", IS_ZERO_UPDATE, 0);
    nh.param<double>("L_B_0", L_B_0, 0);
    nh.param<double>("L_B_1", L_B_1, 0);   
    nh.param<double>("L_B_2", L_B_2, 0);

    nh.param<int>("IS_GPS_VELOCITY_UPDATE", IS_GPS_VELOCITY_UPDATE, 1);
    nh.param<int>("IS_GPS_POSITION_UPDATE", IS_GPS_POSITION_UPDATE, 1);
    nh.param<int>("IS_VERBOSE_MODE", IS_VERBOSE_MODE, 0);

    nh.param<double>("INIT_ROLL", INIT_ROLL, 0);
    nh.param<double>("INIT_PITCH", INIT_PITCH, 0);
    nh.param<double>("INIT_YAW", INIT_YAW, 0);
    
    nh.param<double>("GYRO_STD_0", GYRO_STD_0, 0);
    nh.param<double>("GYRO_STD_1", GYRO_STD_1, 0);
    nh.param<double>("GYRO_STD_2", GYRO_STD_2, 0);
    
    nh.param<double>("ACC_STD_0", ACC_STD_0, 0);
    nh.param<double>("ACC_STD_1", ACC_STD_1, 0);
    nh.param<double>("ACC_STD_2", ACC_STD_2, 0);

    nh.param<double>("GYRO_BIAS_STD_0", GYRO_BIAS_STD_0, 0);
    nh.param<double>("GYRO_BIAS_STD_1", GYRO_BIAS_STD_1, 0);
    nh.param<double>("GYRO_BIAS_STD_2", GYRO_BIAS_STD_2, 0);

    nh.param<double>("ACC_BIAS_STD_0", ACC_BIAS_STD_0, 0);
    nh.param<double>("ACC_BIAS_STD_1", ACC_BIAS_STD_1, 0);
    nh.param<double>("ACC_BIAS_STD_2", ACC_BIAS_STD_2, 0);

    nh.param<double>("GPS_POS_STD_0", GPS_POS_STD_0, 100);
    nh.param<double>("GPS_POS_STD_1", GPS_POS_STD_1, 100);
    nh.param<double>("GPS_POS_STD_2", GPS_POS_STD_2, 100);

    nh.param<double>("GPS_VEL_STD_0", GPS_VEL_STD_0, 10);
    nh.param<double>("GPS_VEL_STD_1", GPS_VEL_STD_1, 10);
    nh.param<double>("GPS_VEL_STD_2", GPS_VEL_STD_2, 10);



}

bool SimpleEKF::InititalizeFilter(const Eigen::Vector3d& accBias, Eigen::Vector3d& gyroBias){

    Eigen::Matrix3d R0;
    Eigen::Vector3d v0, p0, na, ng, nba, nbg, g;
    Eigen::MatrixXd Cov = Eigen::MatrixXd::Zero(15,15);

    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(INIT_ROLL, Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(INIT_PITCH, Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(INIT_YAW, Eigen::Vector3d::UnitZ()));

    R0 = yawAngle * pitchAngle * rollAngle;
    v0 << 0, 0, 0;
    p0 = R0 * Eigen::Vector3d(L_B_0,L_B_1,L_B_2);
    na << ACC_STD_0, ACC_STD_1, ACC_STD_2;
    nba << ACC_BIAS_STD_0, ACC_BIAS_STD_1, ACC_BIAS_STD_2;
    ng << GYRO_STD_0, GYRO_STD_1, GYRO_STD_2;
    nbg << GYRO_BIAS_STD_0, GYRO_BIAS_STD_1, GYRO_BIAS_STD_2;
    g << 0, 0, -9.8015;
    
    // Set covariance.
    Cov.block<3, 3>(0, 0) = 5.0 * deg_to_rad * 5.0 * deg_to_rad * Eigen::Matrix3d::Identity();
    Cov.block<3, 3>(3, 3) = 0.01 * Eigen::Matrix3d::Identity(); // velocity std: 0.1 m/s
    Cov.block<3, 3>(6, 6) = 1 * Eigen::Matrix3d::Identity(); // position std: 
   
    // Acc bias cov
    Cov.block<3, 3>(9, 9) = 0.0004 * Eigen::Matrix3d::Identity();
    // Gyro bias cov
    Cov.block<3, 3>(12, 12) = 0.0004 * Eigen::Matrix3d::Identity();

    CarState state;
    state.setP(Cov);
    state.setRotation(R0);
    state.setVelocity(v0);
    state.setPosition(p0);
    state.setGyroscopeBias(gyroBias);
    state.setGyroscopeNoise(ng);
    state.setGyroscopeBiasNoise(nbg);
    state.setAccelerometerBias(accBias);
    state.setAccelerometerNoise(na);
    state.setAccelerometerBiasNoise(nba);
    state.setGravity(g);

    filter_ = new Ekf(state);
    return true;
}

void SimpleEKF::PubState(){
    
    Eigen::Quaterniond q(filter_->getState().getRotation());
    Eigen::Vector3d p(filter_->getState().getPosition());
    
    //Publish transforms
    odometryTransform.frame_id_ = "/imu_init";
    odometryTransform.child_frame_id_ = "/imu";
    odometryTransform.stamp_ = sysTime; 
    odometryTransform.setOrigin(tf::Vector3(p(0), p(1), p(2)));
    odometryTransform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    tfBroadcaster.sendTransform(odometryTransform);

    odometryTransform.frame_id_  = "/imu_init";
    odometryTransform.child_frame_id_ = "/velodyne";
    tfBroadcaster.sendTransform(odometryTransform);

    // Publish trajactory
    static nav_msgs::Path pathMsg;
    geometry_msgs::PoseStamped pose;

    pose.header.frame_id = "/imu_init";
    pose.header.stamp = sysTime;
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    pose.pose.position.x = p(0);
    pose.pose.position.y = p(1);
    pose.pose.position.z = p(2);

    pathMsg.header.stamp = sysTime; 
	pathMsg.header.frame_id="/imu_init"; 
    pathMsg.poses.push_back(pose);
    pathPub.publish(pathMsg);

    Eigen::Vector3d vb = filter_->getState().getRotation().transpose() * filter_->getState().getVelocity();
    double sideangle = atan2(vb(1), vb(0)) * 180.0 / 3.1415920;
    sensor_msgs::Imu msg;
    msg.header.stamp = sysTime; 
    msg.angular_velocity.z = sideangle;
    resultPub.publish(msg);

    if (IS_VERBOSE_MODE){
        // Stream out state
        // cout << "rotation" << endl;
        // cout << filter_->getState().getRotation() << endl;

        // cout << "position:" << endl;
        // cout << filter_->getState().getPosition() << endl;

        // cout << "velocity:" << endl;
        // cout << filter_->getState().getVelocity() << endl;

        // cout << "Acc bias is:" << endl; 
        // cout << filter_->getState().getAccelerometerBias() << endl;

        // cout << "Gyro bias is:" << endl; 
        // cout << filter_->getState().getGyroscopeBias() << endl;
                
        // cout << "P is" << endl;
        // cout << filter_->getState().getP() << endl << endl;

        // cout << "X is" << endl;
        // cout << filter_->getState().getX() << endl << endl;
        // Eigen::Vector3d vb = filter_->getState().getRotation().transpose() * filter_->getState().getVelocity();
        // double sideangle = atan2(vb(1), vb(0)) * 180.0 / 3.1415920;
        // cout << sideangle << endl;
    }
}

void SimpleEKF::ImuHandler(const sensor_msgs::ImuConstPtr& imuMsg){
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
    sysTime = imuMsg->header.stamp; // Update system time for next step
    imuGlobal = *imuMsg; // update imu msg for global use
    //Load Imu data
    static Eigen::Matrix<double,6,1> imuMeasurement = Eigen::Matrix<double,6,1>::Zero();  
    imuMeasurement << imuMsg->angular_velocity.x,    imuMsg->angular_velocity.y,    imuMsg->angular_velocity.z,
                      imuMsg->linear_acceleration.x, imuMsg->linear_acceleration.y, imuMsg->linear_acceleration.z;
    //Propagate filter 
    filter_->Propagate(imuMeasurement, dt);
    //Pubstates
    PubState();
}

void SimpleEKF::GpsFrontHandler(const sensor_msgs::NavSatFixConstPtr& gpsMsg){

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
    if (IS_GPS_POSITION_UPDATE && (sysTime.toSec() - initTime.toSec()) < 10000){ //&& (sysTime.toSec() - initTime.toSec()) < 50 

        Eigen::MatrixXd H;
        Eigen::VectorXd Z;
        Eigen::MatrixXd R;

        Eigen::Vector3d gps_pos_noise(GPS_POS_STD_0*GPS_POS_STD_0, GPS_POS_STD_1*GPS_POS_STD_1, GPS_POS_STD_2*GPS_POS_STD_2);
        Eigen::MatrixXd Ngps = gps_pos_noise.asDiagonal(); 
        posENU = posENU + filter_->getState().getRotation() * Eigen::Vector3d(L_B_0, L_B_1, L_B_2);
        gpsCovarianceType = int(gpsMsg->position_covariance_type);

        // Fill out H and Z and R
        H.conservativeResize(3, 15);
        H.block(0, 0, 3, 15) = Eigen::MatrixXd::Zero(3, 15);
        H.block(0, 6, 3, 3) = -Eigen::Matrix3d::Identity();

        Z.conservativeResize(3, Eigen::NoChange);
        Z.segment(0,3) =  posENU - filter_->getState().getPosition();
        // Eigen::MatrixXd N_PP7 = Eigen::Map<const Eigen::Matrix3d>(gpsMsg->position_covariance.data());
        filter_->Correct(H,Z,Ngps);
    }
    // Velocity Correct
    if (IS_GPS_VELOCITY_UPDATE){ // && ((sysTime.toSec() - initTime.toSec()) < 100)

        Eigen::Vector3d gpsVelocity;  // in NED frame
        Eigen::Vector3d angularVelocity;
        angularVelocity << imuGlobal.angular_velocity.x, imuGlobal.angular_velocity.y, imuGlobal.angular_velocity.z;
        gpsVelocity << gpsMsg->position_covariance[5], gpsMsg->position_covariance[4], -gpsMsg->position_covariance[6];
        // gpsVelocity -= filter_->getState().getRotation() * angularVelocity.cross(Eigen::Vector3d(0, 0 ,0)); 
        Eigen::MatrixXd H;
        Eigen::VectorXd Z;
        Eigen::MatrixXd N;

        Eigen::Vector3d gps_pos_noise(GPS_VEL_STD_0*GPS_VEL_STD_0, GPS_VEL_STD_1*GPS_VEL_STD_1, GPS_VEL_STD_2*GPS_VEL_STD_2);
        Eigen::MatrixXd Ngps = gps_pos_noise.asDiagonal(); 

        // Fill out H and Z
        H.conservativeResize(3, 15);
        H.block(0, 0, 3, 15) = Eigen::MatrixXd::Zero(3, 15);
        H.block(0, 3, 3, 3) =  -Eigen::Matrix3d::Identity();
        
        Z.conservativeResize(3, Eigen::NoChange);
        Z.segment(0,3) = gpsVelocity - filter_->getState().getVelocity();

        // Fill out N
        N = Ngps;
        filter_->Correct(H,Z,N);
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

void SimpleEKF::GpsRearHandler(const sensor_msgs::NavSatFixConstPtr& gpsMsg){

    static nav_msgs::Path pathMsg;
    static sensor_msgs::NavSatFix lastGps;
    static geometry_msgs::PoseStamped pose;
    
    Eigen::Vector3d lla(gpsMsg->latitude, gpsMsg->longitude, llaOrigin(2));
    Eigen::Vector3d posENU;
    ConvertLLAToENU(llaOrigin, lla, &posENU);

    // Pub GPS path for compare
    pose.header.frame_id = "gps_rear";
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
    gpsRearPathPub.publish(pathMsg);

}

void SimpleEKF::WheelSpeedHandler(const can_driver::steerWheel_angleConstPtr& wheelMsg){
    if (!isFilterInitialed) return;
    double rl, rr;
    rl = wheelMsg->wheelSpeed_RL / 295.0 ;
    rr = wheelMsg->wheelSpeed_RR / 295.0 ;
    if(rl <= 0.01 && rr <= 0.01 && sysTimeFlag){
        Eigen::MatrixXd H;
        Eigen::Vector3d Z;
        Eigen::MatrixXd R;
        // correct ACC bias
        Z << imuGlobal.linear_acceleration.x, imuGlobal.linear_acceleration.y, imuGlobal.linear_acceleration.z;
        Z = Z - filter_->getState().getAccelerometerBias() + filter_->getState().getGravity(); 
        H.conservativeResize(3, 15);
        H = Eigen::MatrixXd::Zero(3, 15);
        H.block(0, 12, 3, 3) =  Eigen::Matrix3d::Identity();
        R = 0.001 * Eigen::Matrix3d::Identity();
        // filter_->Correct(H,Z,R);

        // correct GYRO bias
        Z << imuGlobal.angular_velocity.x, imuGlobal.angular_velocity.y, imuGlobal.angular_velocity.z;
        Z = Z - filter_->getState().getGyroscopeBias(); 
        H.conservativeResize(3, 15);
        H = Eigen::MatrixXd::Zero(3, 15);
        H.block(0, 9, 3, 3) =  Eigen::Matrix3d::Identity();
        R = 0.001 * Eigen::Matrix3d::Identity();
        filter_->Correct(H,Z,R);
        return;
    }

    if (IS_ZERO_UPDATE && abs(rl-rr) <0.05){
        Eigen::MatrixXd H;
        Eigen::Vector2d Z;
        Eigen::MatrixXd R;
        H.conservativeResize(2, 15);
        H.block(0, 0, 2, 15) = Eigen::MatrixXd::Zero(2, 15);
        H.block(0, 3, 2, 2) =  filter_->getState().getRotation().block<2,2>(1,1);
        Eigen::Vector3d temp = filter_->getState().getRotation().transpose() * filter_->getState().getVelocity();
        Z.segment(0,2) =  temp.segment(1,2);
        R = 1 * Eigen::Matrix2d::Identity();
        filter_->Correct(H,Z,R);
    }

        // // ZUPT Correct
        // if (IS_ZERO_UPDATE){

        //     Eigen::MatrixXd H;
        //     Eigen::VectorXd Z;
        //     Eigen::MatrixXd N;
        //     Eigen::Vector2d gps_pos_noise(GPS_VEL_STD_0*GPS_VEL_STD_0, GPS_VEL_STD_1*GPS_VEL_STD_1);
        //     Eigen::MatrixXd Ngps = gps_pos_noise.asDiagonal(); 

        //     // Fill out H and Z
        //     H.conservativeResize(2, 15);
        //     H.block(0, 0, 2, 15) = Eigen::MatrixXd::Zero(2, 15);
        //     H.block(0, 3, 2, 2) =  filter_->getState().getRotation().block<2,2>(1,1);
        //     Eigen::Vector3d temp = filter_->getState().getRotation() * filter_->getState().getVelocity();
        //     H.block(0, 0, 2, 2) =  -skew(temp).block<2,2>(1,1);
      
        //     Z.conservativeResize(2, Eigen::NoChange);
        //     Z.segment(0,2) =  -temp.segment(1,2);

        //     // Fill out N
        //     N = 10000 * Ngps;
        //     filter_->Correct(H,Z,N);
        // }


//     if (IS_WHEEL_UPDATE && abs(rl-rr) <0.05 && rl > 5){
//         filter->LeftToRight();
//         Eigen::MatrixXd H;
//         Eigen::VectorXd Z;
//         Eigen::MatrixXd N;
            
//         Eigen::Vector3d temp = filter->getState().getRotation().transpose() * filter->getState().getVelocity();
//         Z.conservativeResize(3, Eigen::NoChange);
//         Z = -temp;
//         Z(0) -= 0.5 * (rl+ rr);

//         H.conservativeResize(3,15);
//         H.block(0, 0, 3, 15) = Eigen::MatrixXd::Zero(3, 15);
//         H.block(0, 3, 3, 3) = filter->getState().getRotation().transpose().block(0, 0 ,3, 3);
        
//         N.conservativeResize(3, 3);
//         N = 1 * Eigen::Matrix3d::Identity();
        
  
//         filter->RightCorrect(H, Z, N);

//         filter->RightToLeft();
//     }
} 

void SimpleEKF::NovatelPvxHandler(const novatel_gps_msgs::InspvaxConstPtr& pvxMsg){
     
    // double x = (pvxMsg->longitude - gpsOrigin(1)) * 5105840.41295 *0.01745329252;
    // double y = (pvxMsg->latitude - gpsOrigin(0)) * 6378137 *0.01745329252;
    // double z = (pvxMsg->altitude - gpsOrigin(2));

    // Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(pvxMsg->roll*deg_to_rad, Eigen::Vector3d::UnitX()));
    // Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(pvxMsg->pitch*deg_to_rad, Eigen::Vector3d::UnitY()));
    // Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd((360 -pvxMsg->azimuth +90)*deg_to_rad, Eigen::Vector3d::UnitZ()));

    // Eigen::Matrix3d R;
    // R = yawAngle * pitchAngle * rollAngle;
    // Eigen::Quaterniond q(R);
    // Eigen::Vector3d p(x,y,z);
    
    //Publish transforms
    // odometryTransform.frame_id_  = "/imu_init";
    // odometryTransform.child_frame_id_ = "/novatel_pva";
    // odometryTransform.stamp_ = sysTime; 
    // odometryTransform.setOrigin(tf::Vector3(p(0), p(1), p(2)));
    // odometryTransform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    // tfBroadcaster.sendTransform(odometryTransform);

    // Eigen::Vector3d velo = p + R * Eigen::Vector3d(1.64,0,0);
    // odometryTransform.setOrigin(tf::Vector3(velo(0), velo(1), velo(2)));
    // odometryTransform.frame_id_  = "/imu_init";
    // odometryTransform.child_frame_id_ = "/velodyne";
    // tfBroadcaster.sendTransform(odometryTransform);

    // // Publish trajactory
    // static nav_msgs::Path pathMsg;
    // geometry_msgs::PoseStamped pose;

    // pose.header.frame_id = "/imu_init";
    // pose.header.stamp = sysTime;
    // pose.pose.orientation.x = q.x();
    // pose.pose.orientation.y = q.y();
    // pose.pose.orientation.z = q.z();
    // pose.pose.orientation.w = q.w();

    // pose.pose.position.x = p(0);
    // pose.pose.position.y = p(1);
    // pose.pose.position.z = p(2);

    // pathMsg.header.stamp = sysTime; 
	// pathMsg.header.frame_id="/imu_init"; 
    // pathMsg.poses.push_back(pose);
    // pathPub.publish(pathMsg);
}


int main(int argc, char** argv){

    ros::init(argc, argv, "simple_ekf");
    ROS_INFO("start simple ekf node ");

    SimpleEKF simpleEKF;
    ros::Rate rate(1000);

    while (ros::ok()){   
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
