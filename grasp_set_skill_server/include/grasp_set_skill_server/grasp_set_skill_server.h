
/**\file grasp_set_skill_server.h
 * \brief File with grasp_set_skill class definition
 *
 * @version 1.0
 * @author João Pedro Carvalho de Souza
 */

#ifndef GRASP_SET_SKILL_SERVER
#define GRASP_SET_SKILL_SERVER

#pragma once

// ROS includes
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <grasp_set_skill_msgs/GraspSetAction.h>
#include <grasp_set_skill_msgs/GraspCandidate.h>
#include <grasp_set_skill_msgs/GraspCandidateArr.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


// Std includes
//...

namespace grasp_set_skill {
    class GraspSet {
    public:
        typedef actionlib::SimpleActionServer<grasp_set_skill_msgs::GraspSetAction> GraspSetActionServer;

        GraspSet ();

        ~GraspSet (void);

        void start(); //start the server
        void executeCB (const grasp_set_skill_msgs::GraspSetGoalConstPtr &goal); // recognize the goal request
        bool executeProcess(); // execute the procedure after the goal request
        void feedback (float percentage);
        void setSucceeded (std::string outcome = "succeeded");
        void setAborted (std::string outcome = "aborted");
        bool checkPreemption ();

        bool setupSkillConfigurationFromParameterServer(ros::NodeHandlePtr &_node_handle,
                                                        ros::NodeHandlePtr &_private_node_handle);

        enum GRIPPER_ID {       // Type of gripper used
            SUCTION,          // our simple and small suction gripper
            ROBOTIQ_2F_C40666 // two adaptive fingers gripper from Robotiq
        };

    protected:

        ros::NodeHandlePtr node_handle_;
        ros::NodeHandlePtr private_node_handle_;

        std::shared_ptr<GraspSetActionServer>                               actionServer_; // to not init it in the constructor
        std::string                                                         action_server_name_;
        grasp_set_skill_msgs::GraspSetFeedback                              feedback_;
        grasp_set_skill_msgs::GraspSetResult                                result_;

        int number_of_candidates_;

        bool readData();

        bool pubData();
        bool checkParameters();
        bool checkParameters(std::string _input);

        tf2_ros::StaticTransformBroadcaster static_broadcaster_;
        //std::vector<grasp_set_skill_msgs::GraspCandidate> candidates_; // create the TF based on a file
        grasp_set_skill_msgs::GraspCandidateArr grasp_candidates_arr_;
        std::string candidates_grasps_tf_base_name_,
                    object_tf_origin_name_,
                    object_namespace_;

    };
}
#endif // GRASP_SET_SKILL_SERVER
