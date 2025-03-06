from cognitive_nodes.pnode import PNode
import random

class DummyPNode(PNode):
    """
    Activated Dummy PNode class
    """
    def __init__(self, name='pnode', class_name='cognitive_nodes.pnode.PNode', space_class=None, space=None, **params):
        """
        Constructor for the Activated Dummy PNode class

        Initializes a PNode that is always activated and registers it in the ltm

        :param name: The name of the Dummy PNode.
        :type name: str
        :param class_name: The name of the Dummy PNode class.
        :type class_name: str
        :param space_class: The class of the space used to define the Dummy PNode
        :type space_class: str
        :param space: The space used to define the Dummy PNode
        :type space: cognitive_nodes.space
        """
        super().__init__(name, class_name, space_class, space, **params)

    def send_pnode_space_callback(self, request, response):
        response.labels = []
        response.data = []
        response.confidences = []

        return response

    def calculate_activation(self, perception=None, activation_list=None):
        """
        Define the activation in the subclass

        :param perception: The perception for which PNode activation is calculated (does not influence)
        :type perception: dictionary
        :return: Returns the activation of the PNode.
        :rtype: float
        """
        raise NotImplementedError



class ActivatedDummyPNode(DummyPNode):
    """
    Activated Dummy PNode class
    """
    def __init__(self, name='pnode', class_name='cognitive_nodes.pnode.PNode', space_class=None, space=None, **params):
        """
        Constructor for the Activated Dummy PNode class

        Initializes a PNode that is always activated and registers it in the ltm

        :param name: The name of the Dummy PNode.
        :type name: str
        :param class_name: The name of the Dummy PNode class.
        :type class_name: str
        :param space_class: The class of the space used to define the Dummy PNode
        :type space_class: str
        :param space: The space used to define the Dummy PNode
        :type space: cognitive_nodes.space
        """
        super().__init__(name, class_name, space_class, space, **params)

    def calculate_activation(self, perception=None, activation_list=None):
        """
        Always returns an activation of 1.0

        :param perception: The perception for which PNode activation is calculated (does not influence)
        :type perception: dictionary
        :return: Returns the activation of the PNode. Always 1.0
        :rtype: float
        """
        self.activation.activation = 1.0
        self.activation.timestamp = self.get_clock().now().to_msg()
        return self.activation
    
class NonActivatedDummyPNode(DummyPNode):
    """
    Activated Dummy PNode class
    """
    def __init__(self, name='pnode', class_name='cognitive_nodes.pnode.PNode', space_class=None, space=None, **params):
        """
        Constructor for the Activated Dummy PNode class

        Initializes a PNode that is always activated and registers it in the ltm

        :param name: The name of the Dummy PNode.
        :type name: str
        :param class_name: The name of the Dummy PNode class.
        :type class_name: str
        :param space_class: The class of the space used to define the Dummy PNode
        :type space_class: str
        :param space: The space used to define the Dummy PNode
        :type space: cognitive_nodes.space
        """
        super().__init__(name, class_name, space_class, space, **params)

    def calculate_activation(self, perception=None, activation_list=None):
        """
        Always returns an activation of 1.0

        :param perception: The perception for which PNode activation is calculated (does not influence)
        :type perception: dictionary
        :return: Returns the activation of the PNode. Always 1.0
        :rtype: float
        """
        self.activation.activation = 0.0
        self.activation.timestamp = self.get_clock().now().to_msg()
        return self.activation
    
class RandomDummyPNode(DummyPNode):
    """
    Random Dummy PNode class
    """
    def __init__(self, name='pnode', class_name='cognitive_nodes.pnode.PNode', space_class=None, space=None, **params):
        """
        Constructor for the Random Dummy PNode class

        Initializes a PNode which activation is random and registers it in the ltm

        :param name: The name of the Dummy PNode.
        :type name: str
        :param class_name: The name of the Dummy PNode class.
        :type class_name: str
        :param space_class: The class of the space used to define the Dummy PNode
        :type space_class: str
        :param space: The space used to define the Dummy PNode
        :type space: cognitive_nodes.space
        """
        super().__init__(name, class_name, space_class, space, **params)

    def calculate_activation(self, perception=None, activation_list=None):
        """
        Return a random activation

        :param perception: The perception for which PNode activation is calculated (does not influence)
        :type perception: dictionary
        :return: Returns the activation of the PNode. Random value
        :rtype: float
        """
        self.activation.activation = random.random()
        self.activation.timestamp = self.get_clock().now().to_msg()
        return self.activation