from cognitive_nodes.pnode import PNode
from cognitive_nodes.random_utils import get_rng

from core_interfaces.msg import Container as ContainerMsg

class DummyPNode(PNode):
    """
    Activated Dummy PNode class
    """

    def send_pnode_space_callback(self, request, response):
        """
        Callback that sends the space of the P-Node.

        :param request: Empty request.
        :type request: cognitive_node_interfaces.srv.SendGoalSpace.Request
        :param response: Response that contains the space of the P-Node. 
            In this case it is empty.
        :type response: cognitive_node_interfaces.srv.SendGoalSpace.Response
        :return: Response that contains the space of the P-Node. 
            In this case it is empty.
        :rtype: cognitive_node_interfaces.srv.SendGoalSpace.Response
        """     
        response.space = ContainerMsg()  # Return an empty space
        return response

    def calculate_activation(self, perception=None, activation_list=None):
        """
        Activation value for the dummy P-Nodes.
        This method has to be implemented in a subclass.

        :param perception: The perception for which P-Node activation is calculated.
        :type perception: dict
        :param activation_list: The list of activations to be used for the calculation.
        :type activation_list: list
        :raises NotImplementedError: If the method is not implemented in a subclass.
        """
        raise NotImplementedError



class ActivatedDummyPNode(DummyPNode):
    """
    Activated Dummy PNode class
    """
    def calculate_activation(self, perception=None, activation_list=None):
        """
        Always returns an activation of 1.0

        :param perception: The perception for which P-Node activation is calculated.
            It is not used in this case.
        :type perception: dict
        :param activation_list: The list of activations to be used for the calculation.
            It is not used in this case.
        :type activation_list: list
        :return: A msg with the activation of the P-Node and its timestamp.
        :rtype: cognitive_node_interfaces.msg.Activation
        """
        self.activation.activation = 1.0
        self.activation.timestamp = self.get_clock().now().to_msg()
        return self.activation
    
class NonActivatedDummyPNode(DummyPNode):
    """
    Activated Dummy PNode class
    """
    def calculate_activation(self, perception=None, activation_list=None):
        """
        Always returns an activation of 0.0

        :param perception: The perception for which P-Node activation is calculated.
            It is not used in this case.
        :type perception: dict
        :param activation_list: The list of activations to be used for the calculation.
            It is not used in this case.
        :type activation_list: list
        :return: A msg with the activation of the P-Node and its timestamp.
        :rtype: cognitive_node_interfaces.msg.Activation
        """
        self.activation.activation = 0.0
        self.activation.timestamp = self.get_clock().now().to_msg()
        return self.activation
    
class RandomDummyPNode(DummyPNode):
    """
    Random Dummy PNode class
    """
    def calculate_activation(self, perception=None, activation_list=None):
        """
        Return a random activation

        :param perception: The perception for which P-Node activation is calculated.
            It is not used in this case.
        :type perception: dict
        :param activation_list: The list of activations to be used for the calculation.
            It is not used in this case.
        :type activation_list: list
        :return: A msg with the activation of the P-Node and its timestamp.
        :rtype: cognitive_node_interfaces.msg.Activation
        """
        if not hasattr(self, "rng"):
            self.rng = get_rng(getattr(self, "random_seed", None))
        self.activation.activation = float(self.rng.random())
        self.activation.timestamp = self.get_clock().now().to_msg()
        return self.activation