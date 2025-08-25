import yaml
import inspect
import numpy
from rclpy.node import Node
from rclpy import spin_until_future_complete
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.time import Time

from core.service_client import ServiceClient, ServiceClientAsync
from core_interfaces.srv import AddNodeToLTM, DeleteNodeFromLTM, UpdateNeighbor, CreateNode
from cognitive_node_interfaces.srv import GetActivation, GetInformation, SetActivationTopic, AddNeighbor, DeleteNeighbor
from cognitive_node_interfaces.msg import Activation
from core.utils import perception_msg_to_dict

class CognitiveNode(Node):
    """
    A base class for cognitive nodes in the system.

    This class extends the `rclpy.node.Node` class and provides some
    common functionality for cognitive nodes.
    """

    def __init__(self, name, class_name, **params):
        """
        Initialize a CognitiveNode.

        :param name: The name of the node.
        :type name: str
        :param class_name: The name of the class, e.g., 'cognitive_nodes.perception.Perception'.
        :type class_name: str
        """
        super().__init__(name)
        self.name = name
        self.class_name = class_name
        _, _, node_type = self.class_name.rpartition(".")
        self.node_type = node_type

        self.perception = None

        self.neighbors = [] # List of dics, like [{"name": "pnode1", "node_type": "PNode"}, {"name": "cnode1", "node_type": "CNode"}]
        
        #List that contains subscribers of the activation of the node's neighbors
        self.activation_inputs={}
        self.activation_topic = True
        self.activation = Activation()
        self.activation.node_name=self.name
        self.activation.node_type=self.node_type

        self.perception = []
        self.threshold = 0.0

        for key, value in params.items():
            setattr(self, key, value)

        #Callback groups to separate between service requests, service calls and activation callbacks
        self.cbgroup_server=MutuallyExclusiveCallbackGroup()
        self.cbgroup_client=MutuallyExclusiveCallbackGroup()
        self.cbgroup_activation=MutuallyExclusiveCallbackGroup()

        self.node_clients={} #Keys are service name, values are service client object e.g. {'cognitive_node/policy0/get_activation: "Object: Node.client"'}

        # Publish node activation topic (when SetActivationTopic is true)
        self.publish_activation_topic = self.create_publisher(
            Activation,
            'cognitive_node/' + str(name) + '/activation',
            0
        )

        # Get Activation Service
        self.get_activation_service = self.create_service(
            GetActivation,
            'cognitive_node/' + str(name) + '/get_activation',
            self.get_activation_callback, callback_group=self.cbgroup_activation
        )
        
        # Get Information Service
        self.get_information_service = self.create_service(
            GetInformation,
            'cognitive_node/' + str(name) + '/get_information',
            self.get_information_callback, callback_group=self.cbgroup_server
        )
        
        # Set Activation Topic Service
        self.set_activation_service = self.create_service(
            SetActivationTopic,
            'cognitive_node/' + str(name) + '/set_activation_topic',
            self.set_activation_topic_callback, callback_group=self.cbgroup_server
        )

        #Add Neighbor Service
        self.add_neighbor_service = self.create_service(
            AddNeighbor,
            'cognitive_node/' + str(name) + '/add_neighbor',
            self.add_neighbor_callback, callback_group=self.cbgroup_server
        )

        #Delete Neighbor Service
        self.delete_neighbor_service = self.create_service(
            DeleteNeighbor,
            'cognitive_node/' + str(name) + '/delete_neighbor',
            self.delete_neighbor_callback, callback_group=self.cbgroup_server
        )

        #Periodic publishing of activation
        self.activation_publish_timer=self.create_timer(0.01, self.publish_activation_callback, callback_group=self.cbgroup_server)

        #Service clients to add or delete nodes from the LTM
        service_name_add_LTM = 'ltm_0' + '/add_node' # TODO choose LTM ID
        self.add_node_to_LTM_client = ServiceClientAsync(self, AddNodeToLTM, service_name_add_LTM, self.cbgroup_client)
        service_name_delete_LTM = 'ltm_0' + '/delete_node' # TODO: choose the ltm ID
        self.delete_node_client = ServiceClientAsync(self, DeleteNodeFromLTM, service_name_delete_LTM, self.cbgroup_client)
    
    def get_data(self):
        """
        Get the data associated with the node.

        This method returns a dictionary containing the attributes of
        the node, excluding private attributes and the 'subscription'
        attribute found in the ANode class.

        :return: A dictionary with node data.
        :rtype: dict
        """

        node_data = self.__dict__.copy()
        
        # delete any key starting with '_', any topic and any service.
        keys_to_delete = [key for key in node_data.keys() if key.startswith('_') or 'service' in key or 'topic' in key]
        for key in keys_to_delete:
            del node_data[key]

        # list of other keys to delete (filled manually)
        optional_keys_to_delete = ['subscription']
        for key in optional_keys_to_delete:
            if key in node_data:
                del node_data[key]

        return node_data
    
    async def register_in_LTM(self, data_dic):
        """
        Requests registering the node in the LTM. 

        :param data_dic: A dictionary with the data to be saved.
        :type data_dic: dict
        :return: A future that will contain the response from the LTM service.
        :rtype: rclpy.task.Future
        """
        self.get_logger().debug(f'DEBUG START Registering {self.node_type} {self.name} in LTM...')
        
        data = yaml.dump({**data_dic, 'activation': self.activation.activation, 'activation_timestamp': Time.from_msg(self.activation.timestamp).nanoseconds, 'neighbors': self.neighbors})

        ltm_response = self.add_node_to_LTM_client.send_request_async(name=self.name, node_type=self.node_type, data=data)
        await ltm_response
        self.get_logger().debug(f'DEBUG FINISH Registering {self.node_type} {self.name} in LTM...')

        return ltm_response
    
    def remove_from_LTM(self):
        """
        Removes the node from the LTM. 
        :return: True if the operation was succesful, False otherwise.
        :rtype: core_interfaces.srv.DeleteNodeFromLTM.Response
        """
        ltm_response = self.delete_node_client.send_request_async(name=self.name)
        return ltm_response.deleted
   
    def calculate_activation(self, perception, activation_list):
        """
        Calculate the node's activation for the given perception.
        :param perception: The perception for which the activation will be calculated.
        :type perception: dict
        :param activation_list: List of activations considered in the node
        :type activation_list: dict
        """
        raise NotImplementedError
    
    def extract_oldest_timestamp(self, activation_list):
        """
        Helper method to obtain the oldest timestamp from all the activations that the node has recieved.

        :param activation_list: Dictionary with the activation of multiple nodes. 
        :type activation_list: dict
        :return: Timestamp message with the oldest activation and the node that produced it.
        :rtype: Tuple (builtin_interfaces.msg.Time, str)
        """        
        timestamp_dict={node_name: Time.from_msg(activation_list[node_name]['data'].timestamp).nanoseconds for node_name in activation_list}
        if timestamp_dict:
            oldest_node = min(zip(timestamp_dict.values(), timestamp_dict.keys()))[1]
            oldest_timestamp = activation_list[oldest_node]['data'].timestamp
        else:
            oldest_node = None
            oldest_timestamp = Time().to_msg() 
        return (oldest_timestamp, oldest_node) 

    def calculate_activation_prod(self, activation_list):
        """
        Calculates the activation of the node by multiplying the activation of the nodes in the activation_list. 
        The timestamp of the resulting activation will be the oldest timestamp of the nodes in the list.

        :param activation_list: Dictionary with the activation of multiple nodes. 
        :type activation_list: dict
        """        
        node_activations = [activation_list[node_name]['data'].activation for node_name in activation_list]
        timestamp, _ = self.extract_oldest_timestamp(activation_list)
        if len(node_activations)!=0:
            activation=numpy.prod(node_activations)
        else:
            self.get_logger().debug(f'Node activation list empty!!')
            activation=0.0
        self.activation.activation=float(activation)
        self.activation.timestamp=timestamp

    def calculate_activation_max(self, activation_list):
        """
        Calculates the activation of the node by multiplying the activation of the nodes in the activation_list. 
        The timestamp of the resulting activation will be the oldest timestamp of the nodes in the list.

        :param activation_list: Dictionary with the activation of multiple nodes. 
        :type activation_list: dict
        """        
        node_activations = [activation_list[node_name]['data'].activation for node_name in activation_list]
        timestamp, _ = self.extract_oldest_timestamp(activation_list)
        if len(node_activations)!=0:
            activation=numpy.max(node_activations)
        else:
            self.get_logger().debug(f'Node activation list empty!!')
            activation=0
        self.activation.activation=float(activation)
        self.activation.timestamp=timestamp

    def publish_activation(self, activation: Activation):
        """
        Publish the activation of this node.
        :param activation: The activation to be published.
        :type activation: cognitive_node_interfaces.msg.Activation
        """
        self.publish_activation_topic.publish(activation)
        self.get_logger().debug("Activation for " + str(activation.node_type) + str(activation.node_name) +
                            ": " + str(activation.activation))
    
    def add_neighbor_callback(self, request, response):
        """
        Add a neighbor to the nodes neighbors collection.

        :param request: The request that contains the neighbor info.
        :type request: cognitive_node_interfaces.srv.AddNeighbor.Request
        :param response: The response that indicates if the neighbor was added.
        :type response: cognitive_node_interfaces.srv.AddNeighbor.Response
        :return: The response that indicates if the neighbor was added.
        :rtype: cognitive_node_interfaces.srv.AddNeighbor.Response
        """
        
        node_name = request.neighbor_name
        node_type = request.neighbor_type
        self.get_logger().debug(f'Adding {node_type} {node_name} as neighbor of {self.node_type} {self.name}')
        neighbor = {'name':node_name, 'node_type':node_type}
        self.neighbors.append(neighbor)
        self.create_activation_input(neighbor)
        response.added = True
        return response
    
    def delete_neighbor_callback(self, request, response):
        """
        Delete a neighbor to the nodes neighbors collection.

        :param request: The request that contains the neighbor info.
        :type request: cognitive_node_interfaces.srv.DeleteNeighbor.Request
        :param response: The response that indicates if the neighbor was deleted.
        :type response: cognitive_node_interfaces.srv.DeleteNeighbor.Response
        :return: The response that indicates if the neighbor was deleted.
        :rtype: cognitive_node_interfaces.srv.DeleteNeighbor.Response
        """
        node_name = request.neighbor_name
        node_type = request.neighbor_type
        neighbor_to_delete = {'name':node_name, 'node_type':node_type}

        for neighbor in self.neighbors:
            if neighbor == neighbor_to_delete:
                self.neighbors.remove(neighbor)
                self.delete_activation_input(neighbor_to_delete)
                response.deleted = True
            else:
                response.deleted = False
        
        return response

        

    async def get_activation_callback(self, request, response): 
        """
        Callback method to calculate and return the node's activations.
        This method calculates the activation of the node based on its perception.

        :param request: The request containing the perception data.
        :type request: cognitive_node_interfaces.srv.GetActivation.Request
        :param response: The response that will contain the calculated activation.
        :type response: cognitive_node_interfaces.srv.GetActivation.Response
        :return: The response with the calculated activation.
        :rtype: cognitive_node_interfaces.srv.GetActivation.Response
        """
        self.get_logger().debug('Getting node activation...')
        perception = perception_msg_to_dict(request.perception)
        if inspect.iscoroutinefunction(self.calculate_activation):
            await self.calculate_activation(perception)
        else:
            self.calculate_activation(perception)
        response.activation = float(self.activation.activation)
        return response

    def get_information_callback(self, request, response):
        """
        Callback method to get information about the node.

        This method retrieves information about the node, such as its current activation.
        The activation value is included in the response for external queries.

        :param request: The request for node information.
        :type request: cognitive_node_interfaces.srv.GetInformation.Request
        :param response: The response that will contain the node's information.
        :type response: cognitive_node_interfaces.srv.GetInformation.Response
        :return: The response with the node's information.
        :rtype: cognitive_node_interfaces.srv.GetInformation.Response
        """
        self.get_logger().debug('Getting node information...')
        response.node_name = self.name
        response.node_type = self.node_type
        response.current_activation = self.activation.activation
        response.neighbors_name = [neighbor["name"] for neighbor in self.neighbors]
        response.neighbors_type = [neighbor["node_type"] for neighbor in self.neighbors]
        self.get_logger().debug("The type of the node " + str(response.node_name) + "is " + str(response.node_type) +
                              ". Its last activation is: " + str(response.current_activation) +
                               ". It's neighbors are: " + str(response.neighbors_name) + ". The node" +
                               "type of each neighbor is: " + str(response.neighbors_type))
        return response

    def set_activation_topic_callback(self, request, response):
        """
        Callback method to control activation topic publishing for the node.

        This method toggles the activation topic publishing for the node based on the provided request.

        :param request: True to publish the activation; False otherwise.
        :type request: cognitive_node_interfaces.srv.SetActivationTopic.Request
        :param response: True if the node will publish the activation; False otherwise.
        :type response: cognitive_node_interfaces.srv.SetActivationTopic.Response
        :return: True if the node will publish the activation; False otherwise.
        :rtype: cognitive_node_interfaces.srv.SetActivationTopic.Response
        """
        activation_topic = request.activation_topic
        self.get_logger().info('Setting activation topic to ' + str(activation_topic) + '...')
        self.activation_topic = activation_topic
        response.activation_topic = activation_topic
        return response
    
    async def publish_activation_callback(self):
        """
        Timed publish of the activation value. This method will calculate the activation based on the neighbor's activation, and then publish it in the corresponding topic.
        """        
        if self.activation_topic:
            if len(self.activation_inputs)==0: #Calculates activation when there are no inputs configured (Support for custom nodes)
                updated=True
            else:
                self.get_logger().debug(f'Activation Inputs: {str(self.activation_inputs)}')
                updated= all((self.activation_inputs[node_name]['updated'] for node_name in self.activation_inputs)) 

            if updated:
                if inspect.iscoroutinefunction(self.calculate_activation):
                    await self.calculate_activation(perception=None, activation_list=self.activation_inputs)
                else:
                    self.calculate_activation(perception=None, activation_list=self.activation_inputs)
                for node_name in self.activation_inputs:
                    self.activation_inputs[node_name]['updated']=False
            self.publish_activation(self.activation)

    def create_activation_input(self, node: dict):
        """
        Adds a node to the activation inputs list.

        :param node: Dictionary with the information of the node {'name': <name>, 'node_type': <node_type>}.
        :type node: dict
        """        
        name=node['name']
        node_type=node['node_type']
        if node_type != 'Perception':
            if name not in self.activation_inputs:
                subscriber=self.create_subscription(Activation, 'cognitive_node/' + str(name) + '/activation', self.read_activation_callback, 1, callback_group=self.cbgroup_activation)
                data=Activation()
                updated=False
                new_input=dict(subscriber=subscriber, node_type=node_type, data=data, updated=updated)
                self.activation_inputs[name]=new_input
                self.get_logger().debug(f'Created new activation input: {name} of type {node_type}')
            else:
                self.get_logger().error(f'Tried to add {name} to activation inputs more than once')
    
    def delete_activation_input(self, node: dict):
        """
        Deletes a node from the activation inputs list.

        :param node: Dictionary with the information of the node {'name': <name>, 'node_type': <node_type>}.
        :type node: dict
        """ 
        name=node['name']
        if name in self.activation_inputs:
            self.destroy_subscription(self.activation_inputs[name]['subscriber'])
            self.activation_inputs.pop(name)
    
    def configure_activation_inputs(self, neighbor_list):
        """
        Populates the activation list from a list of neighbors.

        :param neighbor_list: Dictionary with the information of the node [{'name': <name>, 'node_type': <node_type>}, .... ].
        :type neighbor_list: dict
        """ 
        for neighbor in neighbor_list:
            self.create_activation_input(neighbor)

    def read_activation_callback(self, msg: Activation):
        """
        Callback to read the activation of a neighbor node.

        :param msg: Activation message from a neighbor node.
        :type msg: cognitive_node_interfaces.msg.Activation
        """
        node_name=msg.node_name
        if node_name in self.activation_inputs:
            if Time.from_msg(msg.timestamp).nanoseconds>Time.from_msg(self.activation_inputs[node_name]['data'].timestamp).nanoseconds:
                self.activation_inputs[node_name]['data']=msg
                self.activation_inputs[node_name]['updated']=True
            elif Time.from_msg(msg.timestamp).nanoseconds<Time.from_msg(self.activation_inputs[node_name]['data'].timestamp).nanoseconds:
                self.get_logger().warn(f'Detected jump back in time, activation of node: {node_name} ({msg.node_type})')
        
    def add_neighbor_client(self, node_name, neighbor_name):
        """
        Client that adds a node as a neighbor of another node.

        :param node_name: Node to which the neighbor will be added.
        :type node_name: str
        :param neighbor_name: Node that will be added as neighbor.
        :type neighbor_name: str
        :return: Future with service's response
        :rtype: Future
        """        
        response=self.update_neighbor_client(node_name, neighbor_name, True)
        return response

    def delete_neighbor_client(self, node_name, neighbor_name):
        """
        Client that deletes a node as a neighbor of another node.

        :param node_name: Node to which the neighbor will be deleted.
        :type node_name: str
        :param neighbor_name: Node that will be deleted as neighbor.
        :type neighbor_name: str
        :return: Future with service's response.
        :rtype: Future
        """    
        response=self.update_neighbor_client(node_name, neighbor_name, False)
        return response

    def update_neighbor_client(self, node_name, neighbor_name, operation):
        """
        Calls the /update_neighbor service in the LTM.

        :param node_name: Node whose neighbor list will be modified.
        :type node_name: str
        :param neighbor_name: Node that will be added/deleted as neighbor.
        :type neighbor_name: str
        :param operation: Selects between adding (True) or deleting (False) neighbor.
        :type operation: bool
        :return: Future with service's response.
        :rtype: Future
        """        
        if getattr(self, "LTM_id", None):
            service_name=f"{self.LTM_id}/update_neighbor"
            if service_name not in self.node_clients:
                self.node_clients[service_name] = ServiceClientAsync(self, UpdateNeighbor, service_name, self.cbgroup_client)
            response= self.node_clients[service_name].send_request_async(node_name=node_name, neighbor_name=neighbor_name, operation=operation)
            return response

    def create_node_client(self, name, class_name, parameters={}):
        """
        This method calls the add node service of the commander.

        :param name: Name of the node to be created.
        :type name: str
        :param class_name: Name of the class to be used for the creation of the node.
        :type class_name: str
        :param parameters: Optional parameters that can be passed to the node, defaults to {}.
        :type parameters: dict
        :return: Success status received from the commander.
        :rtype: bool
        """

        self.get_logger().info("Requesting node creation")
        params_str = yaml.dump(parameters)
        service_name = "commander/create"
        if service_name not in self.node_clients:
            self.node_clients[service_name] = ServiceClientAsync(self, CreateNode, service_name, self.cbgroup_client)
        response = self.node_clients[service_name].send_request_async(
            name=name, class_name=class_name, parameters=params_str
        )
        return response


    def __str__(self):
        """
        Returns a YAML representation of the node's data.

        :return: YAML representation of the node's data.
        :rtype: str
        """
        data = self.get_data()
        return yaml.dump(data, default_flow_style=False)

def main(args=None):
    pass

if __name__ == '__main__':
    main()
