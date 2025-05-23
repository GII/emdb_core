import sys
import yaml
import rclpy
import asyncio
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.time import Time
from rclpy import spin_until_future_complete

from core_interfaces.srv import AddNodeToLTM, DeleteNodeFromLTM, GetNodeFromLTM, ReplaceNodeFromLTM, SetChangesTopic, UpdateNeighbor
from cognitive_node_interfaces.srv import AddNeighbor, DeleteNeighbor
from core.service_client import ServiceClient, ServiceClientAsync

class LTM(Node):
    """
    The Long-Term Memory (LTM) node in the cognitive architecture.

    This node is responsible for storing and managing cognitive nodes of various types.
    It provides services for adding, replacing, deleting, and retrieving these nodes,
    as well as publishing changes.

    Attributes:
        id (str): An identifier for the LTM instance.
        changes_topic (bool): Flag to indicate if changes are being published.
        cognitive_nodes (dict): A dictionary to store cognitive nodes by type.
        state_publisher (Publisher): Publisher for the state of the LTM.
        add_node_service (Service): Service to add new cognitive nodes.
        replace_node_service (Service): Service to replace existing cognitive nodes.
        delete_node_service (Service): Service to delete cognitive nodes.
        update_neighbors_service (Service): Service to update the neighbors list of a node.
        get_node_service (Service): Service to retrieve data of cognitive nodes.
        set_changes_topic_service (Service): Service to set the changes topic.
    """    
    
    def __init__(self, id):
        """
        Initialize the LTM node.

        :param id: The identifier for this LTM instance.
        :type id: int
        """        
        super().__init__('ltm_' + str(id))
        self.id = id
        self.changes_topic = False
        # TODO Create keys from config file
        self.cognitive_nodes = {'CNode': {}, 'Drive': {}, 'Goal': {}, 'Need': {}, 'Policy': {}, 'Perception': {},'PNode': {}, 'UtilityModel': {}, 'WorldModel': {}}

        
        # State topic
        self.state_publisher = self.create_publisher(
            String, 
            'state',
            10
        )

        self.cbgroup_server=MutuallyExclusiveCallbackGroup()
        self.cbgroup_client=MutuallyExclusiveCallbackGroup()

        self.node_clients={}

        # Add node service
        self.add_node_service = self.create_service(
            AddNodeToLTM,
            'ltm_' + str(self.id) + '/add_node',
            self.add_node_callback, callback_group=self.cbgroup_server
        )

        # Replace node service
        self.replace_node_service = self.create_service(
            ReplaceNodeFromLTM,
            'ltm_' + str(self.id) + '/replace_node',
            self.replace_node_callback, callback_group=self.cbgroup_server
        )

        # Delete node service
        self.delete_node_service = self.create_service(
            DeleteNodeFromLTM,
            'ltm_' + str(self.id) + '/delete_node',
            self.delete_node_callback, callback_group=self.cbgroup_server
        )

        # Change connection
        self.update_neighbors_service = self.create_service(
            UpdateNeighbor,
            'ltm_' + str(self.id) + '/update_neighbor',
            self.update_neighbor_callback, callback_group=self.cbgroup_server
        )

        # Get node service
        self.get_node_service = self.create_service(
            GetNodeFromLTM,
            'ltm_' + str(self.id) + '/get_node',
            self.get_node_callback, callback_group=self.cbgroup_server
        )

        # Set changes topic service
        self.set_changes_topic_service = self.create_service(
            SetChangesTopic,
            'ltm_' + str(self.id) + '/set_changes_topic',
            self.set_changes_topic_callback, callback_group=self.cbgroup_server
        )

    def publish_state(self):
        """Publishes the LTM state in the state topic."""        
        if(self.changes_topic):
            msg = String()
            data_dic = self.cognitive_nodes
            data= yaml.dump(data_dic)
            msg.data = data
            self.state_publisher.publish(msg)
            self.get_logger().debug(f"State: {msg.data}")

    # region Properties
    @property
    def drives(self):
        """
        Get all cognitive nodes of type 'Drive' from the LTM.

        :return: A list of 'Drive' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('Drive', [])
    
    @property
    def goals(self):
        """
        Get all cognitive nodes of type 'Goal' from the LTM.

        :return: A list of 'Goal' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('Goal', [])
    
    @property
    def needs(self):
        """
        Get all cognitive nodes of type 'Need' from the LTM.

        :return: A list of 'Need' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('Need', [])
    
    @property
    def policies(self):
        """
        Get all cognitive nodes of type 'Policy' from the LTM.

        :return: A list of 'Policy' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('Policy', [])

    @property
    def pnodes(self):
        """
        Get all cognitive nodes of type 'PNode' from the LTM.

        :return: A list of 'PNode' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('PNode', [])

    @property
    def utilitymodels(self):
        """
        Get all cognitive nodes of type 'UtilityModel' from the LTM.

        :return: A list of 'UtilityModel' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('UtilityModel', [])
    
    @property
    def worldmodels(self):
        """
        Get all cognitive nodes of type 'WorldModel' from the LTM.

        :return: A list of 'WorldModel' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('WorldModel', [])

    # endregion Properties
    
    # region Callbacks
    async def add_node_callback(self, request, response): 
        """
        Callback function for the 'add_node' service.
        Adds a cognitive node to the LTM.

        This method checks if the node already exists in the LTM. If it does, it sets the response
        'added' attribute to False. If the node does not exist, it adds the new node to the LTM 
        and sets the 'added' attribute to True.

        :param request: The service request containing the node's name, type, and data.
        :type request: core_interfaces.srv.AddNodeToLTM.Request
        :param response: The service response.
        :type response: core_interfaces.srv.AddNodeToLTM.Response
        :return: The response indicating whether the node was added successfully.
        :rtype: core_interfaces.srv.AddNodeToLTM.Response
        """
        name = str(request.name)
        node_type = str(request.node_type)
        
        if self.node_exists(node_type, name):
            self.get_logger().info(f"{node_type} {name} already exists.")
            response.added = False

        else:
            data = str(request.data)
            data_dic = yaml.load(data, Loader=yaml.FullLoader)
            await self.add_node(node_type, name, data_dic)   
            self.get_logger().info(f"Added {node_type} {name}")
            response.added = True

        return response
    
    def replace_node_callback(self, request, response):
        """
        Callback function for the 'replace_node' service.
        Replaces an existing cognitive node in the LTM.

        This method first checks if the original node exists in the LTM. If it doesn't, it sets the 
        response 'replaced' attribute to False. If the original node exists, it then checks if the new 
        name is already taken. If not, it replaces the node with the new data and sets the 'replaced' 
        attribute to True.

        :param request: The service request containing the original and new name of the node, its type, 
                and the new data for the node.
        :type request: core_interfaces.srv.ReplaceNodeFromLTM.Request
        :param response: The service response.
        :type response: core_interfaces.srv.ReplaceNodeFromLTM.Response
        :return: The response indicating whether the node was replaced successfully.
        :rtype: core_interfaces.srv.ReplaceNodeFromLTM.Response
        """
        name = str(request.name)
        new_name = str(request.new_name)
        node_type = str(request.node_type)
        
        if not self.node_exists(node_type, name):
            self.get_logger().info(f"{node_type} {name} doesn't exist.")
            response.replaced = False

        elif self.node_exists(node_type, new_name):
            self.get_logger().info(f"{node_type} {name} already exists.")
            response.replaced = False

        else:
            data = str(request.data)
            data_dic = yaml.load(data, Loader=yaml.FullLoader)
            self.add_node(node_type, name, data_dic)     
            self.get_logger().info(f"Replaced {node_type} {name} with {node_type} {name}.")
            response.replaced = True

        return response
    
    def delete_node_callback(self, request, response):
        """
        Callback function for the 'delete_node' service.
        Deletes a cognitive node from the LTM.

        This method iterates over all node types in the LTM to find the node with the given name. 
        If the node is found, it is deleted, and the response 'deleted' attribute is set to True. 
        If the node is not found, the 'deleted' attribute is set to False.

        :param request: The service request containing the name of the node to be deleted.
        :type request: core_interfaces.srv.DeleteNodeFromLTM.Request
        :param response: The service response.
        :type response: core_interfaces.srv.DeleteNodeFromLTM.Response
        :return: The response indicating whether the node was deleted successfully.
        :rtype: core_interfaces.srv.DeleteNodeFromLTM.Response
        """
        name = str(request.name)
        for node_type in self.cognitive_nodes:
            if name in self.cognitive_nodes[node_type]:
                self.delete_node(node_type, name)
                self.get_logger().info(f"{node_type} {name} deleted from LTM.")
                response.deleted = True
                return response

        self.get_logger().info(f"Node {name} doesn't exist.")
        response.deleted = False
        return response
    
    def get_node_callback(self, request, response):
        """
        Callback function for the 'get_node' service.
        Retrieves data of a specific cognitive node from the LTM.

        This method iterates over all node types in the LTM to find the node with the given name. 
        If the node is found, its data is serialized into YAML format and returned in the response. 
        If the node is not found, an empty string is returned.

        :param request: The service request containing the name of the node to retrieve.
        :type request: core_interfaces.srv.GetNodeFromLTM.Request
        :param response: The service response containing the node data if found.
        :type response: core_interfaces.srv.GetNodeFromLTM.Response
        :return: The response with the node data in YAML format or an empty string.
        :rtype: core_interfaces.srv.GetNodeFromLTM.Response
        """        
        name = str(request.name)

        if name == "": #Return dict with all nodes if empty string is passed
            data_dic = self.cognitive_nodes
            data= yaml.dump(data_dic)
            self.get_logger().info(f"Sending all nodes in LTM: {self.id}")
            response.data=data
            return response

        else:
            for node_type in self.cognitive_nodes:
                if name in self.cognitive_nodes[node_type]:
                    data_dic = self.cognitive_nodes[node_type][name]
                    data = yaml.dump(data_dic)
                    self.get_logger().info(f"{node_type} {name}: {data}")
                    response.data = data
                    return response
            self.get_logger().info(f"{node_type} {name} doesn't exist.")
            response.data = ""
            return response
    
    
    def set_changes_topic_callback(self, request, response):
        """
        Callback function for the 'set_changes_topic' service.
        Sets the topic for tracking changes in the LTM.

        :param request: The service request containing the boolean value for the changes topic.
        :type request: core_interfaces.srv.SetChangesTopic.Request
        :param response: The service response confirming the updated changes topic.
        :type response: core_interfaces.srv.SetChangesTopic.Response
        :return: The response with the updated changes topic value.
        :rtype: core_interfaces.srv.SetChangesTopic.Response
        """        
        changes_topic = request.changes_topic
        self.changes_topic = changes_topic
        self.get_logger().info(f"Changes topic set to {changes_topic}")
        response.changes_topic = changes_topic
        return response
    
    async def update_neighbor_callback(self, request, response):
        """
        Callback function for the 'update_neighbor' service.
        Updates the neighbor relationship between two cognitive nodes.

        :param request: The service request containing the node name, neighbor name, and operation type.
        :type request: core_interfaces.srv.UpdateNeighbor.Request
        :param response: The service response indicating the success of the operation.
        :type response: core_interfaces.srv.UpdateNeighbor.Response
        :return: The response indicating whether the neighbor update was successful.
        :rtype: core_interfaces.srv.UpdateNeighbor.Response
        """
        self.get_logger().info(f"Processing neighbor change")
        success=False
        node_name=request.node_name
        neighbor_name=request.neighbor_name
        operation=request.operation
        #Get information from the nodes
        node_dict=self.get_node_dict(node_name)
        neighbor_dict=self.get_node_dict(neighbor_name)
        #Return error if nodes are not found
        if not node_dict or not neighbor_dict:
            if not node_dict:
                self.get_logger().error(f"Node {node_name} not found in LTM")
            if not neighbor_dict:
                self.get_logger().error(f"Node {neighbor_name} not found in LTM")
        else:
            #Perform neighbor operation
            neighbor_type=self.get_node_type(neighbor_name)
            neigh_dict={'name': neighbor_name, 'node_type': neighbor_type}
            if operation:
                if neigh_dict not in node_dict['neighbors']:
                    node_dict['neighbors'].append(neigh_dict)
                    await self.add_neighbor(neighbor_name, neighbor_type, node_name)
                    success=True
                    self.get_logger().info(f"Successfully added {neighbor_name} as neighbor of {node_name}")
                else:
                    success=False
                    self.get_logger().error(f"{neighbor_name} is already a neighbor of {node_name}")
            else:
                i=0
                if neigh_dict in node_dict['neighbors']: 
                    for neighbor in node_dict['neighbors']:
                        if neighbor['name']==neighbor_name:
                            del node_dict['neighbors'][i]
                            success=True
                        else:
                            i+=1
                    await self.delete_neighbor(neighbor_name, neighbor_type, node_name)
                    self.get_logger().info(f"Successfully removed {neighbor_name} as neighbor of {node_name}")
                else:
                    success=False
                    self.get_logger().error(f"{neighbor_name} is not a neighbor of {node_name}")
            self.publish_state()
        response.success=success
        return response
        
    # endregion Callbacks
    
    # region CRUD operations
    async def add_node(self, node_type, node_name, node_data):
        """
        Add a cognitive node to the LTM.

        :param node_type: The type of the cognitive node.
        :type node_type: str
        :param node_name: The name of the cognitive node.
        :type node_name: str
        :param node_data: The dictionary containing the data of the cognitive node.
        :type node_data: dict
        """
        self.cognitive_nodes[node_type][node_name] = node_data
        node_dict={'name': node_name, 'node_type': node_type}
        neighbors=[]

        #If neighbors have not been assiged to the node in creation time, assign neighbors according to type
        if not node_data['neighbors']:
            """
            #REMOVING THIS TO TEST DIRECTED ACTIVATIONS
            #Perceptions are linked to Goals, World Models and Policies
            if node_type=='Perception':
                
                goals= [{'name': goal, 'node_type': 'Goal'} for goal in self.cognitive_nodes['Goal']]
                world_models= [{'name': WM, 'node_type': 'WorldModel'} for WM in self.cognitive_nodes['WorldModel']]
                policies= [{'name': policy, 'node_type': 'Policy'} for policy in self.cognitive_nodes['Policy']]
                neighbors=goals+world_models+policies
                self.cognitive_nodes[node_type][node_name]['neighbors']=neighbors
            """
            #Any other node type is linked to all perceptions
            if node_type!='Perception':
                neighbors=[{'name': perception, 'node_type': 'Perception'} for perception in self.cognitive_nodes['Perception']]
                self.cognitive_nodes[node_type][node_name]['neighbors']=neighbors 

            #Calls AddNode service of the new node to add the required neighbors in node's internal dictionary
            for neighbor in neighbors: 
                await self.add_neighbor(neighbor['name'], neighbor['node_type'], node_name)


        """
        #REMOVING THIS TO TEST DIRECTED ACTIVATIONS
        #Add the new node to the dictionary of the corresponding neighbors
        for neighbor in self.cognitive_nodes[node_type][node_name]['neighbors']:
            neighbor_name=neighbor['name']
            await self.add_neighbor(node_name,node_type,neighbor_name)
            for neighbor_type in self.cognitive_nodes:
                if neighbor_name in self.cognitive_nodes[neighbor_type]:
                    self.cognitive_nodes[neighbor_type][neighbor_name]['neighbors'].append(node_dict)
        """
        self.publish_state()
    
    def delete_node(self, node_type, node_name):
        """
        Delete a cognitive node from the LTM.

        :param node_type: The type of the cognitive node.
        :type node_type: str
        :param node_name: The name of the cognitive node.
        :type node_name: str
        """
        del self.cognitive_nodes[node_type][node_name]
        self.publish_state()


    def node_exists(self, node_type, node_name):
        """
        Check if a cognitive node exists in the LTM.

        :param node_type: The type of the cognitive node.
        :type node_type: str
        :param node_name: The name of the cognitive node.
        :type node_name: str
        :return: True if the node exists, False otherwise.
        :rtype: bool
        """
        if node_type in self.cognitive_nodes:
            return node_name in self.cognitive_nodes[node_type]
        return False
    
    def node_type_exists(self, node_type):
        """
        Check if cognitive nodes of a specific type exist in the LTM.

        :param node_type: The type of cognitive node to check.
        :type node_type: str
        :return: True if the specified type exist, False otherwise.
        :rtype: bool
        """
        return node_type in self.cognitive_nodes
    
    def get_node_dict(self, name, default=None):
        """
        Retrieve the dictionary of a cognitive node by its name.

        This method searches for a cognitive node with the given name across all node types
        in the LTM. If the node is found, its dictionary is returned. If the node is not found,
        the provided default value is returned.

        :param name: The name of the cognitive node to retrieve.
        :type name: str
        :param default: The default value to return if the node is not found, defaults to None.
        :type default: any
        :return: The dictionary of the cognitive node if found, otherwise the default value.
        :rtype: dict or any
        """
        data_dic=default
        for node_type in self.cognitive_nodes:
            if name in self.cognitive_nodes[node_type]:
                data_dic = self.cognitive_nodes[node_type][name]
        return data_dic
    
    def get_node_type(self, name):
        """
        Retrieve the type of a cognitive node by its name.

        This method searches for a cognitive node with the given name across all node types
        in the LTM. If the node is found, its type is returned. If the node is not found,
        None is returned.

        :param name: The name of the cognitive node to retrieve the type for.
        :type name: str
        :return: The type of the cognitive node if found, otherwise None.
        :rtype: str or None
        """
        for node_type in self.cognitive_nodes:
            if name in self.cognitive_nodes[node_type]:
                return node_type
        return None
    
    # endregion CRUD operations

    async def add_neighbor(self, neighbor_name, neighbor_type, service_node_name):
        """
        Add a neighbor to a cognitive node by calling its 'add_neighbor' service.

        :param neighbor_name: The name of the neighbor to be added.
        :type neighbor_name: str
        :param neighbor_type: The type of the neighbor to be added.
        :type neighbor_type: str
        :param service_node_name: The name of the cognitive node whose 'add_neighbor' service will be called.
        :type service_node_name: str
        :return: The result of the service call.
        :rtype: cognitive_node_interfaces.srv.AddNeighbor.Response
        """
        service_name = 'cognitive_node/' + service_node_name + '/add_neighbor'
        if service_name not in self.node_clients:
            self.node_clients[service_name]=ServiceClientAsync(self, AddNeighbor, service_name, callback_group=self.cbgroup_client)
        result = await self.node_clients[service_name].send_request_async(neighbor_name=neighbor_name, neighbor_type=neighbor_type)
        return result
    
    async def delete_neighbor(self, neighbor_name, neighbor_type, service_node_name):
        """
        Remove a neighbor from a cognitive node by calling its 'delete_neighbor' service.

        :param neighbor_name: The name of the neighbor to be removed.
        :type neighbor_name: str
        :param neighbor_type: The type of the neighbor to be removed.
        :type neighbor_type: str
        :param service_node_name: The name of the cognitive node whose 'delete_neighbor' service will be called.
        :type service_node_name: str
        :return: The result of the service call.
        :rtype: cognitive_node_interfaces.srv.DeleteNeighbor.Response
        """
        service_name = 'cognitive_node/' + service_node_name + '/delete_neighbor'
        if service_name not in self.node_clients:
            self.node_clients[service_name]=ServiceClientAsync(self, DeleteNeighbor, service_name, callback_group=self.cbgroup_client)
        result = await self.node_clients[service_name].send_request_async(neighbor_name=neighbor_name, neighbor_type=neighbor_type)
        return result
    

def main(args=None):
    rclpy.init()
    id = int(sys.argv[1])
    ltm = LTM(id)

    try:
        rclpy.spin(ltm)
    except KeyboardInterrupt:
        print(f'Keyboard Interrupt Detected: Shutting down LTM_{id}...')
    finally:
        ltm.destroy_node()

if __name__ == '__main__':
    main()
    