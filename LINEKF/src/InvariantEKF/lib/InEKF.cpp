
#include "InEKF.h"

namespace inekf {

using namespace std;

// ------------ Observation -------------
// Default constructor
Observation::Observation(Eigen::VectorXd& Y, Eigen::VectorXd& b, Eigen::MatrixXd& H, Eigen::MatrixXd& N, Eigen::MatrixXd& PI) :
    Y(Y), b(b), H(H), N(N), PI(PI) {}

// Check if empty
bool Observation::empty() { return Y.rows() == 0; }

ostream& operator<<(ostream& os, const Observation& o) {
    os << "---------- Observation ------------" << endl;
    os << "Y:\n" << o.Y << endl << endl;
    os << "b:\n" << o.b << endl << endl;
    os << "H:\n" << o.H << endl << endl;
    os << "N:\n" << o.N << endl << endl;
    os << "PI:\n" << o.PI << endl;
    os << "-----------------------------------";
    return os;  
} 

// ------------ InEKF -------------
// Default constructor
InEKF::InEKF() : g_((Eigen::VectorXd(3) << 0,0,-9.8015).finished()){}

// Constructor with noise params
InEKF::InEKF(NoiseParams params) : g_((Eigen::VectorXd(3) << 0,0,-9.8015).finished()), noise_params_(params) {}

// Constructor with initial state
InEKF::InEKF(RobotState state) : g_((Eigen::VectorXd(3) << 0,0,-9.8015).finished()), state_(state) {}

// Constructor with initial state and noise params
InEKF::InEKF(RobotState state, NoiseParams params) : g_((Eigen::VectorXd(3) << 0,0,-9.8015).finished()), state_(state), noise_params_(params) {}


// Return robot's current state
RobotState InEKF::getState() { 
    return state_; 
}

// Sets the robot's current state
void InEKF::setState(RobotState state) { 
    state_ = state;
}

// Return noise params
NoiseParams InEKF::getNoiseParams() { 
    return noise_params_; 
}

// Sets the filter's noise parameters
void InEKF::setNoiseParams(NoiseParams params) { 
    noise_params_ = params; 
}

// InEKF Right Propagation - Inertial Data
void InEKF::RightPropagate(const Eigen::Matrix<double,6,1>& m, double dt) {

    Eigen::Vector3d w = m.head(3) - state_.getGyroscopeBias();    // Angular Velocity
    Eigen::Vector3d a = m.tail(3) - state_.getAccelerometerBias(); // Linear Acceleration
    
    Eigen::MatrixXd X = state_.getX();
    Eigen::MatrixXd P = state_.getP();

    // Extract State
    Eigen::Matrix3d R = state_.getRotation();
    Eigen::Vector3d v = state_.getVelocity();
    Eigen::Vector3d p = state_.getPosition();
    
    
    // Strapdown IMU motion model
    Eigen::Vector3d phi = w * dt; 
    Eigen::Matrix3d R_pred = R * Exp_SO3(phi);
    Eigen::Vector3d v_pred = v + (R*a + g_)*dt;
    Eigen::Vector3d p_pred = p + v*dt + 0.5*(R*a + g_)*dt*dt;

    // Set new state (bias has constant dynamics)
    state_.setRotation(R_pred);
    state_.setVelocity(v_pred);
    state_.setPosition(p_pred);

    // ---- Linearized invariant error dynamics -----
    int dimX = state_.dimX();
    int dimP = state_.dimP();
    int dimTheta = state_.dimTheta();
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dimP,dimP);
    // Inertial terms
    A.block<3,3>(3,0) = skew(g_); 
    A.block<3,3>(6,3) = Eigen::Matrix3d::Identity();
    // Bias terms
    A.block<3,3>(0,dimP-dimTheta) = -R;
    A.block<3,3>(3,dimP-dimTheta+3) = -R;
    
    A.block<3,3>(3,9) = -skew(v) * R;
    A.block<3,3>(6,9) = -skew(p) * R;

    // Noise terms
    Eigen::MatrixXd Qk = Eigen::MatrixXd::Zero(dimP,dimP);
    Qk.block<3,3>(0,0) = noise_params_.getGyroscopeCov(); 
    Qk.block<3,3>(3,3) = noise_params_.getAccelerometerCov();
    Qk.block<3,3>(dimP-dimTheta,dimP-dimTheta) = noise_params_.getGyroscopeBiasCov();
    Qk.block<3,3>(dimP-dimTheta+3,dimP-dimTheta+3) = noise_params_.getAccelerometerBiasCov();

    // Discretization
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dimP,dimP);
    Eigen::MatrixXd Phi = I + A*dt; // Fast approximation of exp(A*dt). TODO: explore using the full exp() instead
    Eigen::MatrixXd Adj = I;
    Adj.block(0,0,dimP-dimTheta,dimP-dimTheta) = Adjoint_SEK3(X); // Approx 200 microseconds
    Eigen::MatrixXd PhiAdj = Phi * Adj;
    Eigen::MatrixXd Qk_hat = Adj * Qk * Adj.transpose() * dt; // Approximated discretized noise matrix (faster by 400 microseconds)

    // Propagate Covariance
    // Eigen::MatrixXd P_pred = P  + Qk_hat;
    Eigen::MatrixXd P_pred = Phi * P * Phi.transpose() + Qk_hat;
    // Set new covariance
    state_.setP(P_pred);

    return;
}

void InEKF::LeftPropagate(const Eigen::Matrix<double,6,1>& m, double dt) {

    Eigen::Vector3d w = m.head(3) - state_.getGyroscopeBias();    // Angular Velocity
    Eigen::Vector3d a = m.tail(3) - state_.getAccelerometerBias(); // Linear Acceleration
    
    Eigen::MatrixXd X = state_.getX();
    Eigen::MatrixXd P = state_.getP();
    // Extract State
    Eigen::Matrix3d R = state_.getRotation();
    Eigen::Vector3d v = state_.getVelocity();
    Eigen::Vector3d p = state_.getPosition();
    
    
    // Strapdown IMU motion model
    Eigen::Vector3d phi = w * dt; 
    Eigen::Matrix3d R_pred = R * Exp_SO3(phi);
    Eigen::Vector3d v_pred = v + (R*a + g_)*dt;
    Eigen::Vector3d p_pred = p + v*dt + 0.5*(R*a + g_)*dt*dt;

    // Set new state (bias has constant dynamics)
    state_.setRotation(R_pred);
    state_.setVelocity(v_pred);
    state_.setPosition(p_pred);

    // ---- Linearized invariant error dynamics -----
    int dimX = state_.dimX();
    int dimP = state_.dimP();
    int dimTheta = state_.dimTheta();
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dimP,dimP);
    // Inertial terms
    A.block<3,3>(0,0) = -skew(w); 
    A.block<3,3>(0,9) = -Eigen::Matrix3d::Identity(); 

    A.block<3,3>(3,0) = -skew(a);
    A.block<3,3>(3,3) = -skew(w); 
    A.block<3,3>(3,12) = -Eigen::Matrix3d::Identity(); 

    A.block<3,3>(6,3) = Eigen::Matrix3d::Identity(); 
    A.block<3,3>(6,6) = -skew(w); 

    // Noise terms
    Eigen::MatrixXd Qk = Eigen::MatrixXd::Zero(dimP,dimP); 
    Qk.block<3,3>(0,0) = noise_params_.getGyroscopeCov(); 
    Qk.block<3,3>(3,3) = noise_params_.getAccelerometerCov();

    Qk.block<3,3>(dimP-dimTheta,dimP-dimTheta) = noise_params_.getGyroscopeBiasCov();
    Qk.block<3,3>(dimP-dimTheta+3,dimP-dimTheta+3) = noise_params_.getAccelerometerBiasCov();

    // Discretization
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dimP,dimP);
    Eigen::MatrixXd Phi = I + A*dt; // Fast approximation of exp(A*dt). TODO: explore using the full exp() instead
  
    // Propagate Covariance
    Eigen::MatrixXd P_pred = Phi * P * Phi.transpose() + Qk * dt;
    // Eigen::MatrixXd P_pred = P+ Qk;

    // Set new covariance
    state_.setP(P_pred);
    return;
}

// Correct State: Right-Invariant Observation
void InEKF::Correct(const Observation& obs) {
    // Compute Kalman Gain   
    Eigen::MatrixXd P = state_.getP();
    Eigen::MatrixXd PHT = P * obs.H.transpose();
    Eigen::MatrixXd S = obs.H * PHT + obs.N;
    Eigen::MatrixXd K = PHT * S.inverse();

    // Copy X along the diagonals if more than one measurement
    Eigen::MatrixXd BigX;
    state_.copyDiagX(obs.Y.rows()/state_.dimX(), BigX);
    // cout << BigX;

    // // Compute correction terms
    Eigen::MatrixXd Z = BigX*obs.Y - obs.b;
    Eigen::VectorXd delta = K*obs.PI*Z;
   
    // cout << delta << endl;
    // cout << endl;
    Eigen::MatrixXd dX = Exp_SEK3(delta.segment(0,delta.rows()-state_.dimTheta()));
    Eigen::VectorXd dTheta = delta.segment(delta.rows()-state_.dimTheta(), state_.dimTheta());

    // Update state
    Eigen::MatrixXd X_new = dX*state_.getX(); // Right-Invariant Update
    Eigen::VectorXd Theta_new = state_.getTheta() + dTheta;
    state_.setX(X_new); 
    state_.setTheta(Theta_new);

    // Update Covariance
    Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(state_.dimP(),state_.dimP()) - K*obs.H;
    Eigen::MatrixXd P_new = IKH * P ;

    state_.setP(P_new); 
}   

void InEKF::RightCorrect(const Eigen::MatrixXd H, const Eigen::VectorXd Z, const Eigen::MatrixXd N){
    
    // Compute Kalman Gain
    Eigen::MatrixXd P = state_.getP();
    Eigen::MatrixXd PHT = P * H.transpose();
    Eigen::MatrixXd S = H * PHT + N;
    Eigen::MatrixXd K = PHT * S.inverse();

    // Compute correction terms
    Eigen::VectorXd delta = K*Z;
    Eigen::MatrixXd dX = Exp_SEK3(delta.segment(0,delta.rows()-state_.dimTheta()));
    Eigen::VectorXd dTheta = delta.segment(delta.rows()-state_.dimTheta(), state_.dimTheta());

    
    // Update state
    // Eigen::MatrixXd R_new = Exp_SO3(delta.segment(0,3)).transpose() * state_.getRotation(); // Right-Invariant Update
    // Eigen::VectorXd v_new = Exp_SO3(delta.segment(0,3)).transpose() * (state_.getVelocity() - delta.segment(3,3));
    // Eigen::VectorXd p_new = Exp_SO3(delta.segment(0,3)).transpose() * (state_.getVelocity() - delta.segment(6,3));
    // Eigen::VectorXd Theta_new = state_.getTheta() - dTheta;
    // state_.setRotation(R_new);
    // state_.setVelocity(v_new);
    // state_.setPosition(p_new);
    // state_.setTheta(Theta_new);

    Eigen::MatrixXd X_new = dX*state_.getX(); // Right-Invariant Update
    Eigen::VectorXd Theta_new = state_.getTheta() + dTheta;
    state_.setX(X_new); 
    state_.setTheta(Theta_new);


    // Update Covariance
    Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(state_.dimP(),state_.dimP()) - K*H;
    Eigen::MatrixXd P_new = IKH * P * IKH.transpose() + K*N*K.transpose(); 
    // Eigen::MatrixXd P_new = IKH * P;
    // * IKH.transpose() + K*N*K.transpose(); // Joseph update form
    state_.setP(P_new); 
}

void InEKF::LeftCorrect(const Eigen::MatrixXd H, const Eigen::VectorXd Z, const Eigen::MatrixXd N){
     // Compute Kalman Gain
    Eigen::MatrixXd P = state_.getP();
    Eigen::MatrixXd PHT = P * H.transpose();
    Eigen::MatrixXd S = H * PHT + N;
    Eigen::MatrixXd K = PHT * S.inverse();

    // Compute correction terms
    Eigen::VectorXd delta = K*Z;
    Eigen::MatrixXd dX = Exp_SEK3(delta.segment(0,delta.rows()-state_.dimTheta()));
    Eigen::VectorXd dTheta = delta.segment(delta.rows()-state_.dimTheta(), state_.dimTheta());

    // Update state
    Eigen::MatrixXd X_new = state_.getX() * dX; 
    Eigen::VectorXd Theta_new = state_.getTheta() + dTheta;
    state_.setX(X_new); 
    state_.setTheta(Theta_new);

    // Update Covariance
    Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(state_.dimP(),state_.dimP()) - K*H;
    Eigen::MatrixXd P_new = IKH * P; 
    state_.setP(P_new); 
}

void InEKF::LeftToRight(void){
    Eigen::MatrixXd AdjX;
    Eigen::MatrixXd P_right;
    
    AdjX = Adjoint_SEK3(state_.getX());
    P_right = state_.getP();
    P_right.block(0, 0, 9 ,9) = AdjX * P_right.block(0, 0, 9, 9) * AdjX.transpose();
    state_.setP(P_right);
}
void InEKF::RightToLeft(void){
    
    Eigen::MatrixXd AdjX;
    Eigen::MatrixXd P_left;
    
    AdjX = Adjoint_SEK3(state_.getX());
    P_left = state_.getP();
    P_left.block(0, 0, 9 ,9) = AdjX.inverse() * P_left.block(0, 0, 9, 9) * AdjX.transpose().inverse();
    
    state_.setP(P_left);
}

} // end inekf namespace
