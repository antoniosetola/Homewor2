#include "kdl_ros_control/kdl_robot.h"
#include "kdl_ros_control/kdl_control.h"
#include "kdl_ros_control/kdl_planner.h"
#include "geometry_msgs/Vector3.h"

#include "kdl_parser/kdl_parser.hpp"
#include "urdf/model.h"
#include <std_srvs/Empty.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/SetModelConfiguration.h"


// Global variables
std::vector<double> jnt_pos(7,0.0), jnt_vel(7,0.0), obj_pos(6,0.0),  obj_vel(6,0.0);
bool robot_state_available = false;


// Functions
KDLRobot createRobot(std::string robot_string)
{
    KDL::Tree robot_tree;
    urdf::Model my_model;
    if (!my_model.initFile(robot_string))
    {
        printf("Failed to parse urdf robot model \n");
    }
    if (!kdl_parser::treeFromUrdfModel(my_model, robot_tree))
    {
        printf("Failed to construct kdl tree \n");
    }
    
    KDLRobot robot(robot_tree);
    return robot;
}

void jointStateCallback(const sensor_msgs::JointState & msg)
{
    robot_state_available = true;
    jnt_pos.clear();
    jnt_vel.clear();
    for (int i = 0; i < msg.position.size(); i++)
    {
        jnt_pos.push_back(msg.position[i]);
        jnt_vel.push_back(msg.velocity[i]);
    }
}

// Main
int main(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("Please, provide a path to a URDF file...\n");
        return 0;
    }

    // Init node
    ros::init(argc, argv, "kdl_ros_control_node");
    ros::NodeHandle n;

    // Rate
    ros::Rate loop_rate(500);

    // Subscribers
    ros::Subscriber joint_state_sub = n.subscribe("/iiwa/joint_states", 1, jointStateCallback);

    // Publishers
    ros::Publisher joint1_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_1_effort_controller/command", 1);
    ros::Publisher joint2_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_2_effort_controller/command", 1);
    ros::Publisher joint3_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_3_effort_controller/command", 1);
    ros::Publisher joint4_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_4_effort_controller/command", 1);
    ros::Publisher joint5_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_5_effort_controller/command", 1);
    ros::Publisher joint6_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_6_effort_controller/command", 1);
    ros::Publisher joint7_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_7_effort_controller/command", 1);
    ros::Publisher error_pub = n.advertise<geometry_msgs::Vector3>("/iiwa/end_effector_error", 1);


    // Services
    ros::ServiceClient robot_set_state_srv = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    ros::ServiceClient pauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");

    // Set robot state
    gazebo_msgs::SetModelConfiguration robot_init_config;
    robot_init_config.request.model_name = "iiwa";
    robot_init_config.request.urdf_param_name = "robot_description";
    robot_init_config.request.joint_names.push_back("iiwa_joint_1");
    robot_init_config.request.joint_names.push_back("iiwa_joint_2");
    robot_init_config.request.joint_names.push_back("iiwa_joint_3");
    robot_init_config.request.joint_names.push_back("iiwa_joint_4");
    robot_init_config.request.joint_names.push_back("iiwa_joint_5");
    robot_init_config.request.joint_names.push_back("iiwa_joint_6");
    robot_init_config.request.joint_names.push_back("iiwa_joint_7");
    robot_init_config.request.joint_positions.push_back(0.0);
    robot_init_config.request.joint_positions.push_back(1.57);
    robot_init_config.request.joint_positions.push_back(-1.57);
    robot_init_config.request.joint_positions.push_back(-1.2);
    robot_init_config.request.joint_positions.push_back(1.57);
    robot_init_config.request.joint_positions.push_back(-1.57);
    robot_init_config.request.joint_positions.push_back(-0.37);
    if(robot_set_state_srv.call(robot_init_config))
        ROS_INFO("Robot state set.");
    else
        ROS_INFO("Failed to set robot state.");

    // Messages
    std_msgs::Float64 tau1_msg, tau2_msg, tau3_msg, tau4_msg, tau5_msg, tau6_msg, tau7_msg ;
    std_srvs::Empty pauseSrv;

    // Wait for robot and object state
    while (!(robot_state_available))
    {
        ROS_INFO_STREAM_ONCE("Robot/object state not available yet.");
        ROS_INFO_STREAM_ONCE("Please start gazebo simulation.");
        if (!(robot_set_state_srv.call(robot_init_config)))
            ROS_INFO("Failed to set robot state.");            
        
        ros::spinOnce();
    }

    // Create robot
    KDLRobot robot = createRobot(argv[1]);
    KDL::Jacobian J;
    robot.update(jnt_pos, jnt_vel);
    int nrJnts = robot.getNrJnts();

    // Specify an end-effector 
    robot.addEE(KDL::Frame::Identity());

    // Joints
    KDL::JntArray qd(robot.getNrJnts()),dqd(robot.getNrJnts()),ddqd(robot.getNrJnts());
    dqd.data.setZero();
    ddqd.data.setZero();

    // Torques
    Eigen::VectorXd tau;
    tau.resize(robot.getNrJnts());

    // Update robot
    robot.update(jnt_pos, jnt_vel);

    // Init controller
    KDLController controller_(robot);

    // EE's trajectory initial position
    KDL::Frame init_cart_pose = robot.getEEFrame();
    Eigen::Vector3d init_position(init_cart_pose.p.data);

    // EE trajectory end position
    Eigen::Vector3d end_position;
    end_position << init_cart_pose.p.x(), -init_cart_pose.p.y(), init_cart_pose.p.z();

    // Plan trajectory
    double traj_duration = 10, acc_duration = 1.5, t = 0.0, init_time_slot = 0.0,traj_radius=0.1;
    //KDLPlanner planner(traj_duration, init_position, traj_radius);                      //decomment when using circular trajectory
    KDLPlanner planner(traj_duration, acc_duration, init_position, end_position);     //decomment when using linear trajectory 
    
    // Retrieve the first trajectory point
    vel_profile vel_prof;
    //trajectory_point p = planner.compute_trajectory(t);
    //trajectory_point p = planner.compute_trajectory_circ(t, vel_prof);                  //decomment when using circular trajectory
    trajectory_point p = planner.compute_trajectory_lin(t, vel_prof);                 //decomment when using  linear trajectory

    // Retrieve initial simulation time
    ros::Time begin = ros::Time::now();
    ROS_INFO_STREAM_ONCE("Starting control loop ...");

    // Init trajectory
    KDL::Frame des_pose = KDL::Frame::Identity(); KDL::Twist des_cart_vel = KDL::Twist::Zero(), des_cart_acc = KDL::Twist::Zero();
    des_pose.M = robot.getEEFrame().M;

    while ((ros::Time::now()-begin).toSec() < 2*traj_duration + init_time_slot)
    {
        if (robot_state_available)
        {   
            // Update robot
            robot.update(jnt_pos, jnt_vel);
            J = robot.getEEJacobian();

            // Update time
            t = (ros::Time::now()-begin).toSec();
            //std::cout << "time: " << p.pos[0] << std::endl;
            

            // Extract desired pose
            des_cart_vel = KDL::Twist::Zero();
            des_cart_acc = KDL::Twist::Zero();
            if (t <= init_time_slot) // wait a second
            { 
                //p = planner.compute_trajectory(0.0);
                //p = planner.compute_trajectory_circ(0.0, vel_prof);
                p = planner.compute_trajectory_lin(0.0, vel_prof);
            }
            else if(t > init_time_slot && t <= traj_duration + init_time_slot)
            {   
                des_cart_vel = KDL::Twist(KDL::Vector(p.vel[0], p.vel[1], p.vel[2]),KDL::Vector::Zero());
                des_cart_acc = KDL::Twist(KDL::Vector(p.acc[0], p.acc[1], p.acc[2]),KDL::Vector::Zero());

                //p = planner.compute_trajectory(t-init_time_slot);
                //p = planner.compute_trajectory_circ(t-init_time_slot, vel_prof);
                p = planner.compute_trajectory_lin(t-init_time_slot, vel_prof);

               
            }
            else
            {
                ROS_INFO_STREAM_ONCE("trajectory terminated");
                break;
            }


            des_pose.p = KDL::Vector(p.pos[0],p.pos[1],p.pos[2]);
            
            
            KDL::Frame current_pose = robot.getEEFrame();
            Eigen::Vector3d error_vector = Eigen::Vector3d::Zero();
            error_vector[0] = des_pose.p.x() - current_pose.p.x();
            error_vector[1] = des_pose.p.y() - current_pose.p.y();
            error_vector[2] = des_pose.p.z() - current_pose.p.z();

            geometry_msgs::Vector3 error_msg;
            error_msg.x = 100*error_vector[0];  
            error_msg.y = 100*error_vector[1];
            error_msg.z = 100*error_vector[2];

            
            // inverse kinematics
            qd.data << jnt_pos[0], jnt_pos[1], jnt_pos[2], jnt_pos[3], jnt_pos[4], jnt_pos[5], jnt_pos[6];
            qd = robot.getInvKin(qd, des_pose);

            dqd.data << jnt_vel[0], jnt_vel[1], jnt_vel[2], jnt_vel[3], jnt_vel[4], jnt_vel[5], jnt_vel[6];
            dqd = robot.getDesVel(des_cart_vel, J); 

            // joint space inverse dynamics control
            double Kp = 70, Kd =7;    
            tau = controller_.idCntr(qd, dqd, ddqd, Kp, Kd);

            // // Cartesian space inverse dynamics control
            //double Kp = 70;
            //double Ko = 35;
            //tau = controller_.idCntr(des_pose, des_cart_vel, des_cart_acc, Kp, Ko,2*0.4*sqrt(Kp), 2*0.5*sqrt(Ko));
            //std::cout << "Gravity vector: " << tau_gravity.transpose() << std::endl;

            // Set torques
            //error_msg.data = errr[0];
            tau1_msg.data = tau[0];
            tau2_msg.data = tau[1];
            tau3_msg.data = tau[2];
            tau4_msg.data = tau[3];
            tau5_msg.data = tau[4];
            tau6_msg.data = tau[5];
            tau7_msg.data = tau[6];

            // Publish
            joint1_effort_pub.publish(tau1_msg);
            joint2_effort_pub.publish(tau2_msg);
            joint3_effort_pub.publish(tau3_msg);
            joint4_effort_pub.publish(tau4_msg);
            joint5_effort_pub.publish(tau5_msg);
            joint6_effort_pub.publish(tau6_msg);
            joint7_effort_pub.publish(tau7_msg);
            error_pub.publish(error_msg);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    if(pauseGazebo.call(pauseSrv))
        ROS_INFO("Simulation paused.");
    else
        ROS_INFO("Failed to pause simulation.");

    return 0;
}