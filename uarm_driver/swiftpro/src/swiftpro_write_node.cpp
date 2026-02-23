/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2017, UFactory, Inc.
 * All rights reserved.
 * Author: Roger Cui  <roger@ufactory.cc>	   
 */
#include <string>
#include <mutex>
#include <chrono>
#include <thread>
#include <deque>
#include <memory>
#include <vector>

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <swiftpro/SwiftproState.h>
#include <swiftpro/status.h>
#include <swiftpro/position.h>
#include <swiftpro/angle4th.h>

using namespace std::chrono_literals;
using LockGuard = std::lock_guard<std::mutex>;

void get_position();
class Event;

serial::Serial _serial;
swiftpro::SwiftproState _pos;

std::deque<std::unique_ptr<Event>> event_queue;
std::mutex _mutex;
std::mutex _mutex_pos;

class Event
{
public:
	virtual void action() {};

protected:
	std::string send_command(std::string cmd)
	{
		_serial.flushInput();
		_serial.write(cmd.c_str());
		return _serial.readline(100, "\r\n");
		//return _serial.read(_serial.available());
	}
};

class EventPositionWrite : public Event
{
public:
	EventPositionWrite(const swiftpro::position msg) : Event(), _msg(msg)
	{

	}

	void action() override
	{
		std::string Gcode = "";
		char x[10];
		char y[10];
		char z[10];

		// {
		//LockGuard guard(_mutex_pos);
		//_pos.x = _msg.x;
		//_pos.y = _msg.y;
		//_pos.z = _msg.z;
		// }
		snprintf(x, sizeof(x), "%.2f", _msg.x);
		snprintf(y, sizeof(y), "%.2f", _msg.y);
		snprintf(z, sizeof(z), "%.2f", _msg.z);
		Gcode = (std::string)"G0 X" + x + " Y" + y + " Z" + z + " F10000" + "\r\n";
		ROS_INFO("%s", Gcode.c_str());

		send_command(Gcode.c_str());
	}

	private:
		const swiftpro::position _msg;
};

class EventAngle4th : public Event
{
public:
	EventAngle4th(const swiftpro::angle4th msg) : Event(), _msg(msg)
	{

	}

	void action() override
	{
		std::string Gcode = "";
		char m4[10];
		
		{
			LockGuard guard(_mutex_pos);
			_pos.motor_angle4 = _msg.angle4th;
		}
		snprintf(m4, sizeof(_msg.angle4th), "%.2f", _msg.angle4th);
		Gcode = (std::string)"G2202 N3 V" + m4 + "\r\n";
		ROS_INFO("%s", Gcode.c_str());

		send_command(Gcode.c_str());
	}

	private:
		const swiftpro::angle4th _msg;
};

class EventStatus : public Event
{
public:
	EventStatus(const swiftpro::status msg) : Event(), _msg(msg)
	{

	}

	void action() override
	{
		std::string Gcode = "";

		if (_msg.status == 1)
			Gcode = (std::string)"M17\r\n";   // attach
		else if (_msg.status == 0)
			Gcode = (std::string)"M2019\r\n";
		else
		{
			ROS_INFO("Error:Wrong swiftpro status input");
			return;
		}
		
		{
			LockGuard guard(_mutex_pos);
			_pos.swiftpro_status = _msg.status;
		}
		ROS_INFO("%s", Gcode.c_str());

		send_command(Gcode.c_str());
	}

	private:
		const swiftpro::status _msg;
};

class EventGripper : public Event
{
public:
	EventGripper(const swiftpro::status msg) : Event(), _msg(msg)
	{

	}

	void action() override
	{
		std::string Gcode = "";

		if (_msg.status == 1)
			Gcode = (std::string)"M2232 V1" + "\r\n";
		else if (_msg.status == 0)
			Gcode = (std::string)"M2232 V0" + "\r\n";
		else
		{
			ROS_INFO("Error:Wrong gripper status input");
			return;
		}
		
		{
			LockGuard guard(_mutex_pos);
			_pos.gripper = _msg.status;
		}
		ROS_INFO("%s", Gcode.c_str());

		send_command(Gcode.c_str());
	}

	private:
		const swiftpro::status _msg;
};

class EventPump : public Event
{
public:
	EventPump(const swiftpro::status msg) : Event(), _msg(msg)
	{

	}

	void action() override
	{
		std::string Gcode = "";
		std_msgs::String result;

		if (_msg.status == 1)
			Gcode = (std::string)"M2231 V1" + "\r\n";
		else if (_msg.status == 0)
			Gcode = (std::string)"M2231 V0" + "\r\n";
		else
		{
			ROS_INFO("Error:Wrong pump status input");
			return;
		}
		
		{
			LockGuard guard(_mutex_pos);
			_pos.pump = _msg.status;
		}

		ROS_INFO("%s", Gcode.c_str());

		send_command(Gcode.c_str());
	}

	private:
		const swiftpro::status _msg;
};

class EventGetJoints : public Event
{
public:

	EventGetJoints(ros::Publisher* pub) : Event(), _pub(pub)
	{

	}

	void action() override
	{
		std::vector<float> pos = {0, 0, 0};
		std_msgs::String result;
		result.data = send_command("P2200\r\n");
		
		if(result.data.length() > 3 && result.data.length() < 30){
			//ROS_INFO("%s", result.data.c_str());
			std::stringstream ss(result.data);
			std::string token;

			ss >> token;
			if (token == "ok") {
				while (ss >> token) {
					if (token[0] == 'X' || token[0] == 'B') {
						pos[0] = std::stof(token.substr(1));
					} else if (token[0] == 'Y' || token[0] == 'L') {
						pos[1] = std::stof(token.substr(1));
					} else if (token[0] == 'Z' || token[0] == 'R') {
						pos[2] = std::stof(token.substr(1));
					}
				}
			}

			if(pos[0] != 0 && pos[1] != 0 && pos[2] != 0){
				//ROS_INFO("1: %f, %f, %f", pos[0], pos[1], pos[2]);

				// Correct angles for MoveIt
				pos[0] =   pos[0] - 90.0;
				pos[1] = - pos[1] + 90.0;
				pos[2] =   pos[2];

				//ROS_INFO("2: %f, %f, %f", pos[0], pos[1], pos[2]);

				// Convert to rad
				for(auto& element : pos){
					element = element / 180.0 * M_PI;
				}

				//ROS_INFO("3: %f, %f, %f", pos[0], pos[1], pos[2]);

				sensor_msgs::JointState joint_state;

				// Timestamp
				joint_state.header.stamp = ros::Time::now();

				// Joint names
				joint_state.name.push_back("Joint1");
				joint_state.name.push_back("Joint2");
				joint_state.name.push_back("Joint3");
				//joint_state.name.push_back("Joint4");
				//joint_state.name.push_back("Joint5");
				//joint_state.name.push_back("Joint6");
				//joint_state.name.push_back("Joint7");
				joint_state.name.push_back("Joint8");
				//joint_state.name.push_back("Joint9");

				// Joint positions (example values)
				joint_state.position.push_back(pos[0]);
				joint_state.position.push_back(pos[1]);
				joint_state.position.push_back(pos[2]);
				//joint_state.position.push_back(0);
				//joint_state.position.push_back(0);
				//joint_state.position.push_back(0);
				//joint_state.position.push_back(0);
				joint_state.position.push_back( - pos[1] - pos[2] );
				//joint_state.position.push_back(0);

				_pub->publish(joint_state);

				{
					LockGuard guard(_mutex_pos);
					_pos.motor_angle1 = pos[0];
					_pos.motor_angle2 = pos[1];
					_pos.motor_angle3 = pos[2];
					_pos.motor_angle4 = - pos[1] - pos[2];
				}
			}
		}
	}

private:
	ros::Publisher* _pub;
};



class EventGetPosition : public Event
{
public:

	void action() override
	{
		std::vector<float> pos = {0, 0, 0};
		std_msgs::String result;
		result.data = send_command("P2220\r\n");
		
		if(result.data.length() > 3 && result.data.length() < 30){
			//ROS_INFO("%s", result.data.c_str());
			std::stringstream ss(result.data);
			std::string token;

			ss >> token;
			if (token == "ok") {
				while (ss >> token) {
					if (token[0] == 'X' || token[0] == 'B') {
						pos[0] = std::stof(token.substr(1));
					} else if (token[0] == 'Y' || token[0] == 'L') {
						pos[1] = std::stof(token.substr(1));
					} else if (token[0] == 'Z' || token[0] == 'R') {
						pos[2] = std::stof(token.substr(1));
					}
				}

				{
					LockGuard guard(_mutex_pos);
					_pos.x = pos[0];
					_pos.y = pos[1];
					_pos.z = pos[2];
				}
			}
		}
	}
};


class EventGetLimitSwitch : public Event
{
public:

	void action() override
	{
		int limit_swtich = 0;
		std_msgs::String result;
		result.data = send_command("P2233\r\n");
		
		if(result.data.length() > 3 && result.data.length() < 30){
			//ROS_INFO("%s", result.data.c_str());
			std::stringstream ss(result.data);
			std::string token;

			ss >> token;
			if (token == "ok") {
				while (ss >> token) {
					if (token[0] == 'V') {
						limit_swtich = std::stoi(token.substr(1));					}
				}

				{
					LockGuard guard(_mutex_pos);
					_pos.limit_switch = limit_swtich;
				}
			}
		}
	}
};


/* 
 * Description: callback when receive data from position_write_topic
 * Inputs: 		msg(float)			3 cartesian coordinates: x, y, z(mm)
 * Outputs:		Gcode				send gcode to control swift pro
 */
void position_write_callback(const swiftpro::position& msg)
{
	LockGuard guard(_mutex);
	event_queue.push_back(std::make_unique<EventPositionWrite>(msg));
}


/* 
 * Description: callback when receive data from angle4th_topic
 * Inputs: 		msg(float)			angle of 4th motor(degree)
 * Outputs:		Gcode				send gcode to control swift pro
 */
void angle4th_callback(const swiftpro::angle4th& msg)
{
	LockGuard guard(_mutex);
	event_queue.push_back(std::make_unique<EventAngle4th>(msg));
}


/* 
 * Description: callback when receive data from swiftpro_status_topic
 * Inputs: 		msg(uint8)			status of gripper: attach if 1; detach if 0
 * Outputs:		Gcode				send gcode to control swift pro
 */
void swiftpro_status_callback(const swiftpro::status& msg)
{
	LockGuard guard(_mutex);
	event_queue.push_back(std::make_unique<EventStatus>(msg));
}


/* 
 * Description: callback when receive data from gripper_topic
 * Inputs: 		msg(uint8)			status of gripper: work if 1; otherwise 0
 * Outputs:		Gcode				send gcode to control swift pro
 */
void gripper_callback(const swiftpro::status& msg)
{
	LockGuard guard(_mutex);
	event_queue.push_back(std::make_unique<EventGripper>(msg));
}


/* 
 * Description: callback when receive data from pump_topic
 * Inputs: 		msg(uint8)			status of pump: work if 1; otherwise 0
 * Outputs:		Gcode				send gcode to control swift pro
 */
void pump_callback(const swiftpro::status& msg)
{
	LockGuard guard(_mutex);
	event_queue.push_back(std::make_unique<EventPump>(msg));
}

/* 
 * Node name:
 *	 swiftpro_write_node
 *
 * Topic publish: (rate = 20Hz, queue size = 1)
 *	 swiftpro_state_topic
 *
 * Topic subscribe: (queue size = 1)
 *	 position_write_topic
 *	 swiftpro_status_topic
 *	 angle4th_topic
 *	 gripper_topic
 *	 pump_topic
 */
int main(int argc, char** argv)
{	
	ros::init(argc, argv, "swiftpro_write_node");
	ros::NodeHandle nh;
	swiftpro::SwiftproState swiftpro_state;

	ros::Subscriber sub1 = nh.subscribe("position_write_topic", 1, position_write_callback);
	ros::Subscriber sub2 = nh.subscribe("swiftpro_status_topic", 1, swiftpro_status_callback);
	ros::Subscriber sub3 = nh.subscribe("angle4th_topic", 1, angle4th_callback);
	ros::Subscriber sub4 = nh.subscribe("gripper_topic", 1, gripper_callback);
	ros::Subscriber sub5 = nh.subscribe("pump_topic", 1, pump_callback);
	ros::Publisher  pub1 = nh.advertise<swiftpro::SwiftproState>("SwiftproState_topic", 1);
	ros::Publisher 	pub2 = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
	ros::Rate loop_rate(100);

	{
		LockGuard guard(_mutex);

		try
		{
			_serial.setPort("/dev/ttyACM0");
			_serial.setBaudrate(115200);
			serial::Timeout to = serial::Timeout::simpleTimeout(1000);
			_serial.setTimeout(to);
			_serial.open();
			ROS_INFO_STREAM("Port has been open successfully");
		}
		catch (serial::IOException& e)
		{
			ROS_ERROR_STREAM("Unable to open port");
			return -1;
		}
		
		if (_serial.isOpen())
		{
			ros::Duration(3.5).sleep();				// wait 3.5s
			_serial.write("M2120 V0\r\n");			// stop report position
			ros::Duration(0.1).sleep();				// wait 0.1s
			_serial.write("M17\r\n");				// attach
			ros::Duration(0.1).sleep();				// wait 0.1s
			ROS_INFO_STREAM("Attach and wait for commands");
		}
	}

	auto last_request = std::chrono::high_resolution_clock::now();
	while (ros::ok())
	{

		auto now = std::chrono::high_resolution_clock::now();
		if((now - last_request) > 200ms)
		{
			LockGuard guard(_mutex);
			event_queue.push_front(std::make_unique<EventGetJoints>(&pub2));
			event_queue.push_front(std::make_unique<EventGetPosition>());
			event_queue.push_front(std::make_unique<EventGetLimitSwitch>());
			last_request = now;
		}

		{
			LockGuard guard(_mutex);
			while(!event_queue.empty())
			{
				auto& event = event_queue.front();
				event->action();
				event_queue.pop_front();
			}
		}

		{
			LockGuard guard(_mutex_pos);
			pub1.publish(_pos);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}


