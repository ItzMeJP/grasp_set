/**\file grasp_set_skill_server_node.cpp
 * \brief Template to creation of action skills
 *
 * @version 1.0
 * @author João Pedro Carvalho de Souza
 */

#include <grasp_set_skill_server/grasp_set_skill_server.h>
#include <grasp_set_skill_server/common/verbosity_levels.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_set_skill");

    ros::NodeHandlePtr node_handle(new ros::NodeHandle());
    ros::NodeHandlePtr private_node_handle(new ros::NodeHandle("~"));

    std::string ros_verbosity_level;
    private_node_handle->param("ros_verbosity_level", ros_verbosity_level, std::string("DEBUG"));
    grasp_set_skill::verbosity_levels::setVerbosityLevelROS(ros_verbosity_level);
    grasp_set_skill::GraspSet test;
    test.setupSkillConfigurationFromParameterServer(node_handle, private_node_handle);
    test.start();
    ros::spin();

}
