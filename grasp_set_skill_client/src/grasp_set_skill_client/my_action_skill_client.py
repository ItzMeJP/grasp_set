import sys
import rospy

from task_manager_common.skill_class import SkillSetup, SkillExecution, SkillAnalysis

class MyActionSkillSetup(SkillSetup):
    pass

class MyActionSkillExecution(SkillExecution):
    pass

class MyActionSkillAnalysis(SkillAnalysis):
    def set_outcomes(self, outcomes=None):
        return ['preempted', 'aborted', 'succeeded', "outcome_1", "outcome_2"]
