
/**\file grasp_set_skill_server.cpp
 * \brief File with grasp_set_skill_server class definition
 *
 * @version 1.0
 * @author João Pedro Carvalho de Souza
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <grasp_set_skill_server/grasp_set_skill_server.h>

namespace grasp_set_skill {

    GraspSet::GraspSet (){}

    GraspSet::~GraspSet (){
        //grasp_candidates_arr_.candidates.clear();
        private_node_handle_->deleteParam("number_of_candidates");
    }

    void GraspSet::start() {

        actionServer_ = std::make_shared<GraspSetActionServer>(*node_handle_, action_server_name_,
                                                                          boost::bind(&GraspSet::executeCB,
                                                                                      this, _1));
        actionServer_->start();
    }

    void GraspSet::executeCB (const grasp_set_skill_msgs::GraspSetGoalConstPtr &goal) {
        ROS_INFO_STREAM("Request Object : " << goal->object_name.c_str());
        object_namespace_ = goal->object_name.c_str();
        executeProcess()?setSucceeded():setAborted();
    }

    void GraspSet::setSucceeded (std::string outcome) {
        result_.percentage  = 100;
        result_.skillStatus = action_server_name_;
        result_.skillStatus += ": Succeeded";
        //result_.outcome     = outcome;
        result_.grasp_candidates = grasp_candidates_arr_;
        ROS_INFO_STREAM(action_server_name_ << ": Succeeded");
        actionServer_->setSucceeded(result_);
    }
    void GraspSet::setAborted (std::string outcome) {
        result_.percentage  = 0;
        result_.skillStatus = action_server_name_;
        result_.skillStatus += ": Aborted";
        //result_.outcome     = outcome;
        result_.grasp_candidates = grasp_candidates_arr_;
        ROS_INFO_STREAM(action_server_name_ << ": Aborted");
        actionServer_->setAborted(result_);
    }
    void GraspSet::feedback (float percentage) {
        feedback_.percentage  = percentage;
        feedback_.skillStatus = action_server_name_;
        feedback_.skillStatus += " Executing";
        ROS_INFO_STREAM(action_server_name_ << ": Executing. Percentage" << percentage);
        actionServer_->publishFeedback(feedback_);
    }

    bool GraspSet::checkPreemption () {
        if (actionServer_->isPreemptRequested() || !ros::ok()) {
            result_.percentage  = 0;
            result_.skillStatus = action_server_name_;
            result_.skillStatus += ": Preempted";
            //result_.outcome     = "preempted";
            result_.grasp_candidates = grasp_candidates_arr_;
            ROS_INFO_STREAM(action_server_name_ << ": Preempted");
            actionServer_->setPreempted(result_);
            return true;
        } else {
            return false;
        }
    }

    bool GraspSet::setupSkillConfigurationFromParameterServer(ros::NodeHandlePtr &_node_handle,
                                                                   ros::NodeHandlePtr &_private_node_handle){
        this->node_handle_ = _node_handle;
        this->private_node_handle_ = _private_node_handle;
        private_node_handle_->param<std::string>("action_server_name", action_server_name_, "grasp_set_skill");
        private_node_handle_->param<std::string>("candidates_grasps_tf_base_name", candidates_grasps_tf_base_name_,"grasp_candidate");
        private_node_handle_->param<std::string>("object_tf_origin", object_tf_origin_name_, "object_origin");

        return true;

    }

    bool GraspSet::executeProcess()
    {
        //start he procedure after goal acquisition

        grasp_candidates_arr_.candidates.clear();
        if(!checkParameters() || !readData() || !pubData())
            return false;

        return true;
    }

    bool GraspSet::readData(){

        private_node_handle_->param("number_of_candidates", number_of_candidates_, 3);

        grasp_set_skill_msgs::GraspCandidate candidate;
        candidate.transform_stamped.header.frame_id = object_tf_origin_name_;

        int g;
        std::string current_candidate;
        for (int j = 0; j < number_of_candidates_; ++j) {

            candidate.transform_stamped.header.stamp = ros::Time::now();
            candidate.transform_stamped.child_frame_id = candidates_grasps_tf_base_name_ +"_"+ std::to_string(j);

            current_candidate = "candidate_"+ std::to_string(j);
            checkParameters(current_candidate);

            private_node_handle_->param(object_namespace_ + "/" + current_candidate +"/gripper", g, 0);
            private_node_handle_->param(object_namespace_ + "/" + current_candidate +"/position/x", candidate.transform_stamped.transform.translation.x, 0.0);
            private_node_handle_->param(object_namespace_ + "/" + current_candidate +"/position/y", candidate.transform_stamped.transform.translation.y, 0.0);
            private_node_handle_->param(object_namespace_ + "/" + current_candidate +"/position/z", candidate.transform_stamped.transform.translation.z, 0.0);
            private_node_handle_->param(object_namespace_ + "/" + current_candidate +"/orientation/x", candidate.transform_stamped.transform.rotation.x, 0.0);
            private_node_handle_->param(object_namespace_ + "/" + current_candidate +"/orientation/y", candidate.transform_stamped.transform.rotation.y, 0.0);
            private_node_handle_->param(object_namespace_ + "/" + current_candidate +"/orientation/z", candidate.transform_stamped.transform.rotation.z, 0.0);
            private_node_handle_->param(object_namespace_ + "/" + current_candidate +"/orientation/w", candidate.transform_stamped.transform.rotation.w, 1.0);
            candidate.gripper = g;

            grasp_candidates_arr_.candidates.push_back(candidate);

            ROS_DEBUG_STREAM(current_candidate+" added: [x: " << candidate.transform_stamped.transform.translation.x <<
                                                                   " |y: "<< candidate.transform_stamped.transform.translation.y <<
                                                                   " |z: "<< candidate.transform_stamped.transform.translation.z <<
                                                                   " |rx: "<< candidate.transform_stamped.transform.rotation.x <<
                                                                   " |ry: "<< candidate.transform_stamped.transform.rotation.y <<
                                                                   " |rz: "<< candidate.transform_stamped.transform.rotation.z <<
                                                                   " |rw: "<< candidate.transform_stamped.transform.rotation.w << "]");

        }

        return true;

    }

    bool GraspSet::pubData () {
        for (int i = 0; i < number_of_candidates_; ++i) {
            static_broadcaster_.sendTransform(grasp_candidates_arr_.candidates[i].transform_stamped);
        }

        return true;
    }

    bool GraspSet::checkParameters()
    {
        if (!private_node_handle_->hasParam("number_of_candidates")){
            ROS_ERROR_STREAM("Please set the number of candidates in \"number_of_candidates\" tag.");
            return false;
        }

        std::string s;
        if(!private_node_handle_->searchParam(object_namespace_,s)){
            ROS_ERROR_STREAM("The namespaces ("<< object_namespace_ << ") does not exist in the parameter server.");
            return false;
        }

        return true;

    }

    bool GraspSet::checkParameters (std::string _input) {
        std::string s;
        if(!private_node_handle_->searchParam(_input,s)){
            ROS_ERROR_STREAM("The string ("<< _input << ") does not exist in the parameter server tree. "
                                                        "The number of candidates in the configuration file of ("<< object_namespace_<<") is less than "<<
                                                        number_of_candidates_ << " (\"number_of_candidates\" parameter value).");
            return false;
        }

        return true;

    }


}
