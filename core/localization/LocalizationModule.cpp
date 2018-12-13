#include <localization/LocalizationModule.h>
#include <memory/WorldObjectBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/RobotStateBlock.h>
#include <localization/ParticleFilter.h>
#include "filter/DynamicGridFilter.hh"
#include "filter/GridFilter.hh"
#include <localization/Logging.h>


#define T 30.0

// Boilerplate
// LocalizationModule::LocalizationModule() : tlogger_(textlogger), pfilter_(new ParticleFilter(cache_, tlogger_)) {
LocalizationModule::LocalizationModule() : tlogger_(textlogger), dgfilter_(new DynamicGridFilter(cache_, tlogger_)) {
// LocalizationModule::LocalizationModule() : tlogger_(textlogger), gfilter_(new GridFilter(cache_, tlogger_)) {
  Filter::filter_init ("NA");
}

LocalizationModule::~LocalizationModule() {
  // delete pfilter_;
  dgfilter_->destroy ();
  delete dgfilter_;
  // gfilter_->destroy ();
  // delete gfilter_;
}

// Boilerplate
void LocalizationModule::specifyMemoryDependency() {
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("localization");
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("game_state");
  requiresMemoryBlock("vision_odometry");
}

// Boilerplate
void LocalizationModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(cache_.world_object,"world_objects");
  getOrAddMemoryBlock(cache_.localization_mem,"localization");
  getOrAddMemoryBlock(cache_.frame_info,"vision_frame_info");
  getOrAddMemoryBlock(cache_.robot_state,"robot_state");
  getOrAddMemoryBlock(cache_.game_state,"game_state");
  getOrAddMemoryBlock(cache_.odometry,"vision_odometry");
}


// Load params that are defined in cfglocalization.py
void LocalizationModule::loadParams(LocalizationParams params) {
  params_ = params;
  printf("Loaded localization params for %s\n", params_.behavior.c_str());
}

// Perform startup initialization such as allocating memory
void LocalizationModule::initSpecificModule() {
  reInit();
}

// Initialize the localization module based on data from the LocalizationBlock
void LocalizationModule::initFromMemory() {
  reInit();
}

// Initialize the localization module based on data from the WorldObjectBlock
void LocalizationModule::initFromWorld() {
  reInit();
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];
  // pfilter_->init(self.loc, self.orientation);
  // gfilter_->init(2);
  dgfilter_->init(0, 0.02);
}

// Reinitialize from scratch
void LocalizationModule::reInit() {
  // pfilter_->init(Point2D(-750,0), 0.0f);
  // gfilter_->init(2);
  dgfilter_->init(0, 0.02);
  cache_.localization_mem->player_ = Point2D(-750,0);
  cache_.localization_mem->state = decltype(cache_.localization_mem->state)::Zero();
  cache_.localization_mem->covariance = decltype(cache_.localization_mem->covariance)::Identity();

  KF_ = new ExtendedKalmanFilter();
  // KF_ = new ExtendedKalmanFilter();
}

void LocalizationModule::moveBall(const Point2D& position) {
  // Optional: This method is called when the player is moved within the localization
  // simulator window.
}

void LocalizationModule::movePlayer(const Point2D& position, float orientation) {
  // Optional: This method is called when the player is moved within the localization
  // simulator window.
}

void LocalizationModule::processFrame() {
  auto& ball = cache_.world_object->objects_[WO_BALL];
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];

  // Process the current frame and retrieve our location/orientation estimate
  // from the particle filter
  // pfilter_->processFrame();
  // self.loc = pfilter_->pose().translation;
  // self.orientation = pfilter_->pose().rotation;
  // gfilter_->process();
  // self.loc = gfilter_->current_state().translation;
  // self.orientation = gfilter_->current_state().rotation;
  dgfilter_->process();
  self.loc = dgfilter_->current_state().translation;
  self.orientation = dgfilter_->current_state().rotation;
  printf("Localization Update: x=%2.f, y=%2.f, theta=%2.2f\n", self.loc.x, self.loc.y, self.orientation * RAD_T_DEG);

  log(40, "Localization Update: x=%2.f, y=%2.f, theta=%2.2f", self.loc.x, self.loc.y, self.orientation * RAD_T_DEG);
    
  //TODO: modify this block to use your Kalman filter implementation
  if(ball.seen) {
    // Compute the relative position of the ball from vision readings
    auto relBall = Point2D::getPointFromPolar(ball.visionDistance, ball.visionBearing);

    // Compute the global position of the ball based on our assumed position and orientation
    auto globalBall = relBall.relativeToGlobal(self.loc, self.orientation);

    // Update the ball in the WorldObject block so that it can be accessed in python
    ball.loc = relBall;
    ball.distance = ball.visionDistance;
    ball.bearing = ball.visionBearing;

    if (lastBallX == -1. && lastBallY == -1) {
      ball.absVel.x = 0;
      ball.absVel.y = 0;
    } else {
      ball.absVel.x = (ball.loc.x - lastBallX) ;
      ball.absVel.y = (ball.loc.y - lastBallY) ;
    }

    lastBallX = ball.loc.x;
    lastBallY = ball.loc.y;

    updateState(ball.seen);

    // Update the localization memory objects with localization calculations
    // so that they are drawn in the World window
    cache_.localization_mem->state[0] = ball.loc.x;
    cache_.localization_mem->state[1] = ball.loc.y;
    cache_.localization_mem->covariance = decltype(cache_.localization_mem->covariance)::Identity() * 10000;
  } 
  //TODO: How do we handle not seeing the ball?
  else {
    KF_->reset();
    // cout << "RESET" << endl;
    ball.distance = 10000.0f;
    ball.bearing = 0.0f;
  }
}



void LocalizationModule::updateState(bool ballSeen) {

  auto& ball = cache_.world_object->objects_[WO_BALL];


  //states, pay attention to the first frame!! NOT handled !! 
  // Was automatically handeled earlier because wt, Covt was sent as parameter which was given default value
  if (!KF_->isInitialized()) {
    ExtendedKalmanFilter::Vectornf initWt;
    initWt << ball.loc.x, ball.loc.y, ball.absVel.x, ball.absVel.y;
    KF_->setwt(initWt);

    ExtendedKalmanFilter::Matrixnnf initCovt = Eigen::MatrixXf::Identity(KF_->get_n(), KF_->get_n()) * 100.0f;
    KF_->setCovt(initCovt);

    // Define (Extended) Kalman filter parameters
    static ExtendedKalmanFilter::Matrixnnf At = Eigen::MatrixXf::Identity(KF_->get_n(), KF_->get_n());
    At(0,2) = 1; // Delta T
    At(1,3) = 1; // Delta T

    // cout << At << endl;

    static ExtendedKalmanFilter::Matrixnmf Bt = Eigen::MatrixXf::Zero(KF_->get_n(), KF_->get_m());
    static ExtendedKalmanFilter::Matrixknf Ct = Eigen::MatrixXf::Identity(KF_->get_k(), KF_->get_n());
    static ExtendedKalmanFilter::Matrixnnf Rt = Eigen::MatrixXf::Identity(KF_->get_n(), KF_->get_n()) * 1.0f; // Rt: Transition variance. Should be LOW
    static ExtendedKalmanFilter::Matrixkkf Qt = Eigen::MatrixXf::Identity(KF_->get_k(), KF_->get_k()) * 89.0f; // Qt: Measurement variance
    Qt(2,2) = 158.0f;
    Qt(3,3) = 158.0f;

    KF_->setConstants(At, Bt, Ct, Rt, Qt);
  }

  // State
  if (ballSeen) {
    static ExtendedKalmanFilter::Vectornf wt;
    static ExtendedKalmanFilter::Matrixnnf cov;

    // Actions. Nothing
    static ExtendedKalmanFilter::Vectormf ut = Eigen::VectorXf::Zero(KF_->get_m());
    // assert(sizeof(*ut)/sizeof(*ut[0]) == KF_->m);

    // Sensor measurements
    static ExtendedKalmanFilter::Vectorkf zt = Eigen::VectorXf::Zero(KF_->get_k());
    zt << ball.loc.x, ball.loc.y, ball.absVel.x, ball.absVel.y;

    std::tie(wt, cov) = KF_->algorithm(ut, zt);

    ball.loc.x = wt(0);
    ball.loc.y = wt(1);
    ball.absVel.x = wt(2);
    ball.absVel.y = wt(3);
    ball.sd = Point2D(cov(0,0), cov(1,1));
  
  } else{

    // KF_->updateCovt(1.05);
  }
}
