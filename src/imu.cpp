#include <ros/ros.h>
#include <robosub2018/State.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <sstream>
#include <vector>
#include <regex>

#define _USE_MATH_DEFINES
#include <cmath>

// Intel NUC
string port = "/dev/ttyUSB0";
int baud = 115200


vector<double> eulerToQuaternion(vector<double> euler) {
	/*
	 * roll  = euler[0]
	 * pitch = euler[1]
	 * yaw	 = euler[2]
	 */

	double rollCos = cos(euler[0] / 2);
	double rollSin = sin(euler[0] / 2);

	double pitchCos = cos(euler[1] / 2);
	double pitchSin = sin(euler[1] / 2);

	double yawCos = cos(euler[2] / 2);
	double yawSin = sin(euler[2] / 2);

	double w = (rollCos * pitchCos * yawCos) + (rollSin * pitchSin * yawSin);
	double x = (rollSin * pitchCos * yawCos) + (rollCos * pitchSin * yawSin);
	double y = (rollCos * pitchSin * yawCos) + (rollSin * pitchCos * yawSin);
	double z = (rollCos * pitchCos * yawSin) + (rollSin * pitchSin * yawCos);

	vector<double> quat = {w, x, y, z};
	return quat;
}


vector<double> getIMUData(string command) {
	ser.write(command);
	// readline
	// parse
	// apply regex - '([-\d.]+)'
	
}


int main(int argc, char **argv) {
	
	ros::init(argc, argv, "imu");
	ros::NodeHandle n;

	ros::Publisher imuPublisher = n.advertise<robosub2019::State>("sensors/state", 8);
	robosub2019::State msg;
	try {
		serial::Serial ser;
		ser.setPort(port);
		ser.setBaudrate(baud);
		ser.open();
	}

	msg.header.seq = 0;
	msg.header.frame_id = "imu0";
	msg.header.stamp = ros::Time::now();
	msg.orientation_covariance = {0.000001, 0.0, 0.0, 0.0, 0.000001, 0.0, 0.0, 0.0, 0.000001};
	msg.angular_velocity_covariance = {0.000001, 0.0, 0.0, 0.0, 0.000001, 0.0, 0.0, 0.0, 0.000001};
	msg.linear_acceleration_covariance = {0.00117, 0.0, 0.0, 0.0, 0.00277, 0.0, 0.0, 0.0, 0.00034};
	

	ros::Rate rate(100);
	while(ros::ok()) {
		msg.header.stamp = ros::Time::now();
		msg.header.seq++;

		/*
		 * Magnetometer
		 * Conversions: None
		 */
		double quat[4] = getIMUData("$PSPA,QUAT\r\n");
		msg.orientation.w = quat[0];
		msg.orientation.x = quat[1];
		msg.orientation.y = quat[2];
		msg.orientation.z = quat[3];

		/*
		 * Gyrometer
		 * Conversions: millidegrees -> radians
		 *
		 * NOTE: ROS Imu Message uses 3DOF, ~4DOF
		 * NOTE: Robosub2019 State Message uses 4DOF
		 */
		double gyro[3] = getIMUData("$PSPA,G\r\n");
		msg.angular_velocity.x = gyro[0] * M_PI / 180 / 1000
		msg.angular_velocity.y = gyro[1] * M_PI / 180 / 1000
		msg.angular_velocity.z = gyro[2] * M_PI / 180 / 1000

		/*
		 * Accelerometer
		 * Conversions: milli-Gs -> m/s^2
		 */
		double accel[3] = getIMUData("$PSPA,A\r\n");
		msg.linear_acceleration.x = accel[0] * 9.80665 / 1000;
		msg.linear_acceleration.y = accel[1] * 9.80665 / 1000;
		msg.linear_acceleration.z = accel[2] * 9.80665 / 1000;

		imuPublisher.publish(msg);
		ros::spinOnce();
		rate.sleep();
	}

	ros::waitForShutdown();
	return 0;
}
