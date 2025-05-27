from cognitive_nodes.goal import GoalMotiven

class GoalDummy(GoalMotiven):
    """
    Dummy Goal class: A goal that is never rewarded.
    """

    def calculate_reward(self, drive_name): #No reward is provided
        """
        Calculates the reward of the goal based on the evaluation of the Drive node.
        In this case, the reward is always 0.0.

        :param drive_name: Name of the drive node. Not used in this case.
        :type drive_name: str
        :return: The reward value and the timestamp.
        :rtype: tuple
        """
        self.reward = 0.0
        return self.reward, self.get_clock().now().to_msg()