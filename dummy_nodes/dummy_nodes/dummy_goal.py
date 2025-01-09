from cognitive_nodes.goal import GoalMotiven

class GoalEffectance(GoalMotiven):
    def __init__(self, name='goal', class_name='cognitive_nodes.goal.Goal', **params):
        super().__init__(name, class_name, **params)
    
    def calculate_reward(self, drive_name): #No reward is provided
        self.reward = 0.0
        return self.reward, self.get_clock().now().to_msg()