#include <eeros/hal/RealsenseT265.hpp>
 
using namespace eeros::logger;
using namespace eeros::hal;

RealsenseT265::RealsenseT265(std::string dev, int priority) 
    : Thread(priority), starting(true), running (false), log(Logger::getLogger('P')) {
  cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);  // Add pose stream
  pipe.start(cfg);                                      // Start pipeline with chosen configuration
  starting = false;
}

RealsenseT265::~RealsenseT265() { 
  running = false;
  join();
}

void RealsenseT265::run(){
  while(starting);
  running = true;
  while (running) {
    // Wait for the next set of frames from the camera
    auto frames = pipe.wait_for_frames();
    // Get a frame from the pose stream
    auto f = frames.first_or_default(RS2_STREAM_POSE);
    // Cast the frame to pose_frame and get its data
    auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
    
    // outputs for eeros
    translation     << pose_data.translation.x, pose_data.translation.y, pose_data.translation.z;
    velocity        << pose_data.velocity.x, pose_data.velocity.y, pose_data.velocity.z;
    acceleration    << pose_data.acceleration.x, pose_data.acceleration.y, pose_data.acceleration.z;
    quaternion      << pose_data.rotation.w, pose_data.rotation.x, pose_data.rotation.y, pose_data.rotation.z;
    angVelocity     << pose_data.angular_velocity.x, pose_data.angular_velocity.y, pose_data.angular_velocity.z;
    angAcceleration << pose_data.angular_acceleration.x, pose_data.angular_acceleration.y, pose_data.angular_acceleration.z;
    
    // Calculate current transformation matrix
    float r[16];
    calc_transform(pose_data, r);
    // From the matrix we found, get the new location point
//     rs2_vector tr{ r[12], r[13], r[14] };
    // Create a new point to be added to the trajectory
//     tracked_point p{ tr , pose_data.tracker_confidence };        
  }
}

void RealsenseT265::calc_transform(rs2_pose& pose_data, float mat[16]) {
  auto q = pose_data.rotation;
  auto t = pose_data.translation;
  // Set the matrix as column-major for convenient work with OpenGL and rotate by 180 degress (by negating 1st and 3rd columns)
  mat[0] = -(1 - 2 * q.y*q.y - 2 * q.z*q.z); mat[4] = 2 * q.x*q.y - 2 * q.z*q.w;     mat[8] = -(2 * q.x*q.z + 2 * q.y*q.w);      mat[12] = t.x;
  mat[1] = -(2 * q.x*q.y + 2 * q.z*q.w);     mat[5] = 1 - 2 * q.x*q.x - 2 * q.z*q.z; mat[9] = -(2 * q.y*q.z - 2 * q.x*q.w);      mat[13] = t.y;
  mat[2] = -(2 * q.x*q.z - 2 * q.y*q.w);     mat[6] = 2 * q.y*q.z + 2 * q.x*q.w;     mat[10] = -(1 - 2 * q.x*q.x - 2 * q.y*q.y); mat[14] = t.z;
  mat[3] = 0.0f;                             mat[7] = 0.0f;                          mat[11] = 0.0f;                             mat[15] = 1.0f;
}


