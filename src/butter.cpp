#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

constexpr size_t kQueueSize = 100;

//global variables because I'm lazy
ros::Publisher imu_pub_;

class Butter2 {
 public:
  // constructor
  // Coefficients from http://www-users.cs.york.ac.uk/~fisher/mkfilter
  //

  // filtertype  = Butterworth
  // passtype  = Lowpass
  // ripple  =
  // order = 2
  // samplerate  = 200
  // corner1 = 50
  // corner2 =
  // adzero  =
  // logmin  = -20
  Butter2() {
    // 50 Hz cutoff
    //gain_ = 3.414213562e+00;
    //a_[0] = -0.1715728753;
    //a_[1] = 0.0000000000
    
    // 25 Hz cutoff
    gain_ = 1.024264069e+01;
    a_[0] = -0.3333333333;
    a_[1] = 0.9428090416;
    initalised = false;
  }
  /**
   * Add a new raw value to the filter
   *
   * @return retrieve the filtered result
   */
  double apply(double sample) {

    if(!initalised){
      initalised = true;
      return reset(sample);
    }
    xs_[0] = xs_[1];
    xs_[1] = xs_[2];
    xs_[2] = sample / gain_;
    ys_[0] = ys_[1];
    ys_[1] = ys_[2];
    ys_[2] =
        (xs_[0] + xs_[2]) + 2 * xs_[1] + (a_[0] * ys_[0]) + (a_[1] * ys_[1]);
    return ys_[2];
  }

  /**
   * Reset the filter state to this value
   */
  double reset(double sample) {
    xs_[0] = sample;
    xs_[1] = sample;
    xs_[2] = sample;
    ys_[0] = sample;
    ys_[1] = sample;
    ys_[2] = sample;
    return sample;
  }

 private:
  bool initalised;
  double a_[2];

  double gain_;

  double xs_[3];
  double ys_[3];
};


void imuCallback(const sensor_msgs::ImuConstPtr& msg_in) {

  static Butter2 butter_ax;
  static Butter2 butter_ay;
  static Butter2 butter_az;
  static Butter2 butter_wx;
  static Butter2 butter_wy;
  static Butter2 butter_wz;

  sensor_msgs::Imu msg_out = *msg_in;
  msg_out.linear_acceleration.x = butter_ax.apply(msg_in->linear_acceleration.x);
  msg_out.linear_acceleration.y = butter_ay.apply(msg_in->linear_acceleration.y);
  msg_out.linear_acceleration.z = butter_az.apply(msg_in->linear_acceleration.z);

  msg_out.angular_velocity.x = butter_wx.apply(msg_in->angular_velocity.x);
  msg_out.angular_velocity.y = butter_wy.apply(msg_in->angular_velocity.y);
  msg_out.angular_velocity.z = butter_wz.apply(msg_in->angular_velocity.z);

  imu_pub_.publish(msg_out);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "butter_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ros::Subscriber imu_sub =
      nh.subscribe("input_imu", kQueueSize, &imuCallback);

  imu_pub_ = nh.advertise<sensor_msgs::Imu>("output_imu", kQueueSize);

  ros::spin();

  return 0;
}
