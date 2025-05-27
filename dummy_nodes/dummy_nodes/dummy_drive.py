
from cognitive_nodes.drive import Drive

class DriveDummy(Drive):
    """
    Drive which evaluation is always 1.0.
    """
    def evaluate(self, perception=None):
        """
        Get expected valuation for a given perception.
        In this case, the evaluation is always 1.0.

        :param perception: The given normalized perception. Not used in this case.
        :type perception: dict
        :return: Evaluation msg with the evaluation value and the timestamp.
        :rtype: cognitive_node_interfaces.msg.Evaluation
        """
        self.evaluation.evaluation = 1.0
        self.evaluation.timestamp = self.get_clock().now().to_msg()
        return self.evaluation
    
