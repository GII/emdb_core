
from cognitive_nodes.drive import Drive

class DriveDummy(Drive):
    def __init__(self, name="drive", class_name="cognitive_nodes.drive.Drive", **params):
        super().__init__(name, class_name, **params)
    
    def evaluate(self, perception=None):
        self.evaluation.evaluation = 1.0
        self.evaluation.timestamp = self.get_clock().now().to_msg()
        return self.evaluation