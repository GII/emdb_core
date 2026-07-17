from rclpy.time import Time
import os
import yaml
import threading

from core.service_client import ServiceClient
from core.container import Container
from cognitive_nodes.episodic_buffer import EpisodicBuffer
from cognitive_nodes.episode import container_msg_to_episode


from cognitive_node_interfaces.msg import Activation
from cognitive_node_interfaces.srv import SendSpace, SaveModel
from cognitive_node_interfaces.msg import SuccessRate
from core_interfaces.srv import GetNodeFromLTM

class File():
    """A MDB file."""

    def __init__(self, ident, file_name, node, flush_freq=0, **params):
        """Init attributes when a new object is created."""
        self.ident = ident
        self.file_name = file_name
        self.file_object = None
        self.node = node
        self.write_count = 0
        self.flush_freq = flush_freq

    def __getstate__(self):
        """
        Return the object to be serialize with PyYAML as the result of removing the unpicklable entries.

        :return: A dictionary representing the serializable state of the object.
        :rtype: dict
        """
        state = self.__dict__.copy()
        del state["file_object"]
        return state

    def write_header(self):
        """Write the header of the file."""

        name, extension = os.path.splitext(self.file_name)
        i = 0
        while os.path.exists(f"{name}_{i}{extension}"):
            i = i + 1

        file_name_def = name + "_" + str(i) + extension
        self.file_object = open(file_name_def, "a", encoding="utf-8")

    def close(self):
        """Close de underlying file."""
        if self.file_object:
            self.file_object.close()

    def write(self):
        "Method that writes the data in the file"
        raise NotImplementedError
    
    def write_episode(self, msg):
        "Writes data related to an episode"
        return None

    def _write_file(self, data):
        """Write data to the file."""
        if self.file_object:
            self.file_object.write(data)
            self.write_count += 1
            if self.flush_freq > 0 and self.write_count % self.flush_freq == 0:
                self.file_object.flush()


class FileGoodness(File):
    """A file where several goodness statistics about an experiment are stored."""

    def write_header(self):
        """Write the header of the file."""
        super().write_header()
        self._write_file(
            "Iteration\tWorld\tGoal reward list\tPolicy\tSensorial changes\tC-nodes\n"
        )

    def write(self):
        """Write statistics data."""
        reward_list = self.node.current_episode.reward_list
        if reward_list is None:
            return
        formatted_goals = {goal: f"{reward:.1f}" for goal, reward in sorted(reward_list.items())}
        current_world = self.node.current_world if self.node.current_world else "None"
        self._write_file(
            str(self.node.iteration)
            + "\t"
            + current_world
            + "\t"
            + str(f"{formatted_goals}")
            + "\t"
            + self.node.current_policy
            + "\t"
            + str(self.node.sensorial_changes_val)
            + "\t"
            + str(self.node.n_cnodes)
            + "\n"
        )

        

class FilePNodesSuccess(File):
    """A file that records whether a P-node's activation has been successful or not."""

    def write_header(self):
        """Write the header of the file."""
        super().write_header()
        self.file_object.write("Iteration\tIdent\tSuccess\n")

    def write(self):
        """Write success."""
        for pnode, success in self.node.pnodes_success.items():
            if success is not None:
                self._write_file(
                    str(self.node.iteration)
                    + "\t"
                    + pnode
                    + "\t"
                    + str(success)
                    + "\n"
                )
            self.node.pnodes_success[pnode] = None

class FileTrialsSuccess(File):
    """A file that records whether a P-node's activation has been successful or not."""

    def write_header(self):
        """Write the header of the file."""
        super().write_header()
        self._write_file("Iteration\tTrial\tIterations\tSuccess\n")

    def write(self):
        """Write success."""
        for iteration, trial, iterations, success in self.node.trials_data:
            self._write_file(
                str(iteration)
                + "\t"
                + str(trial)
                + "\t"
                + str(iterations)
                + "\t"
                + str(success)
                + "\n"
            )
            self.file_object.flush()
        self.node.trials_data = []

class FileSpaceContent(File):
    def __init__(self, ident, file_name, node, **params):
        super().__init__(ident, file_name, node, **params)

    def write_header(self):
        """Write the header of the file."""
        super().write_header()
        self.file_object.write("Iteration\tIdent\t")
        self.header_finished = False
        self.created_clients = {}

    def _write_space_content(self, space: Container, ident=""):
        labels = space.feature_labels if space else []

        if labels:
            if not self.header_finished:
                self.finish_header(labels)
            
            if labels == self.labels:
                data = space.read()
                samples = len(space)
                for i in range(samples):
                    self._write_file(str(self.node.iteration) + "\t")
                    self._write_file(ident + "\t")
                    for idx, label in enumerate(labels):
                        ending = "\n" if idx == len(labels) - 1 else "\t"
                        self._write_file(
                            str(data.sel(features=label, sample=i).values) + ending
                        )

            else:
                self._write_file("ERROR. LABELS DO NOT MATCH BETWEEN SPACES\n")

    def finish_header(self, labels):
        """
        Write space dimensions in the header of the file.

        :param labels: Dimensions of the space.
        :type labels: list
        """
        for idx, label in enumerate(labels):
            ending = "\n" if idx == len(labels) - 1 else "\t"
            self.file_object.write(f"{label}{ending}")
        self.header_finished = True
        self.labels = labels

    

class FilePNodesContent(FileSpaceContent):
    """A file that saves the contents of the P-nodes."""   
    def __init__(self, ident, file_name, node, save_interval=100, **params):
        super().__init__(ident, file_name, node, **params)
        self.save_interval = save_interval

    def create_pnode_client(self, pnode_name):
        """
        Create client to request P-Node's space.

        :param pnode_name: Name of the P-Node.
        :type pnode_name: str
        """
        if pnode_name not in self.created_clients:
            pnode_client = ServiceClient(SendSpace, 'pnode/' + str(pnode_name) + '/send_space')
            self.created_clients[pnode_name] = pnode_client

    def write(self):
        """Writes P-Nodes contents."""    
        write_needed = (self.save_interval > 0 and self.node.iteration % self.save_interval == 0) or (self.node.iteration == self.node.iterations)
        if "PNode" in self.node.LTM_cache and write_needed:
            for pnode in self.node.LTM_cache["PNode"]:
                if pnode not in self.created_clients:
                    self.create_pnode_client(pnode)

                if self.created_clients[pnode]:
                    response = self.created_clients[pnode].send_request()
                    space = Container.from_msg(response.space)
                    if space:
                        self._write_space_content(space, ident=pnode)



class FileLastIterationPNodesContent(FilePNodesContent):
    """A file that saves the contents of the P-nodes at the end of an experiment."""
    def write(self):
        """Writes P-Nodes contents"""  
        if "PNode" in self.node.LTM_cache and self.node.iteration == self.node.iterations:
            for pnode in self.node.LTM_cache["PNode"]:
                self.create_pnode_client(pnode)

                if self.created_clients[pnode]:
                    response = self.created_clients[pnode].send_request()
                    space = Container.from_msg(response.space)
                    if space:
                        self._write_space_content(space, ident=pnode)

                    
        
class FileGoalsContent(FileSpaceContent):
    """A file that saves the contents of the Goals at the end of an experiment."""
    def __init__(self, ident, file_name, node, save_interval=100, **params):
        super().__init__(ident, file_name, node, **params)
        self.save_interval = save_interval

    def create_goal_client(self, goal_name):
        """
        Create client to request Goal's space.

        :param goal_name: Name of the Goal.
        :type goal_name: str
        """
        if goal_name not in self.created_clients:
            goal_client = ServiceClient(SendSpace, 'goal/' + str(goal_name) + '/send_space')
            self.created_clients[goal_name] = goal_client


    def write(self):
        """Writes Goals contents.""" 
        if "Goal" in self.node.LTM_cache and self.node.iteration % self.save_interval == 0: #TODO Vary iterations
            for goal in self.node.LTM_cache["Goal"]:
                if goal not in self.created_clients:
                    self.create_goal_client(goal)
                if self.created_clients[goal]:
                    response = self.created_clients[goal].send_request()
                    space = Container.from_msg(response.space)
                    if space:
                        self._write_space_content(space, ident=goal)
    

class FileLastIterationGoalsContent(FileGoalsContent):
    """A file that saves the contents of the Goals at the end of an experiment."""
    def write(self):
        """Writes Goals contents.""" 
        if "Goal" in self.node.LTM_cache and self.node.iteration == self.node.iterations:
            for goal in self.node.LTM_cache["Goal"]:
                self.create_goal_client(goal)
                if self.created_clients[goal]:
                    response = self.created_clients[goal].send_request()
                    space = Container.from_msg(response.space)
                    if space:
                        self._write_space_content(space, ident=goal)



class FileCNodesContent(File):
    """A file that saves C-Node content from the LTM cache."""
    def __init__(self, ident, file_name, node, save_interval=100, **params):
        super().__init__(ident, file_name, node, **params)
        self.save_interval = save_interval

    def write_header(self):
        """Write the header of the file."""
        super().write_header()
        self.file_object.write("Iteration\tIdent\tWorldModel\tPNode\tGoal\tPolicy\tNeighbors\n")

    def _neighbor_by_type(self, neighbors, node_type):
        for neighbor in neighbors:
            if neighbor.get("node_type") == node_type:
                return neighbor.get("name", "")
        return ""

    def _write_cnodes(self, iteration_value):
        for cnode, cnode_data in self.node.LTM_cache["CNode"].items():
            neighbors = cnode_data.get("neighbors", [])
            world_model = self._neighbor_by_type(neighbors, "WorldModel")
            pnode = self._neighbor_by_type(neighbors, "PNode")
            goal = self._neighbor_by_type(neighbors, "Goal")
            policy = self._neighbor_by_type(neighbors, "Policy")
            neighbor_names = [neighbor.get("name", "") for neighbor in neighbors]

            self.file_object.write(
                str(iteration_value)
                + "\t"
                + str(cnode)
                + "\t"
                + str(world_model)
                + "\t"
                + str(pnode)
                + "\t"
                + str(goal)
                + "\t"
                + str(policy)
                + "\t"
                + str(neighbor_names)
                + "\n"
            )

    def write(self):
        """Writes C-Nodes contents."""
        write_needed = (self.save_interval > 0 and self.node.iteration % self.save_interval == 0) or (self.node.iteration == self.node.iterations)
        if "CNode" in self.node.LTM_cache and write_needed:
            self._write_cnodes(self.node.iteration)

class FileLastIterationCNodesContent(FileCNodesContent):
    """A file that saves C-Node content at the end of an experiment."""

    def write(self):
        """Writes C-Nodes contents."""
        if "CNode" in self.node.LTM_cache and self.node.iteration == self.node.iterations:
            self._write_cnodes(self.node.iterations)

class FileNeighbors(File):
    """A file that saves the neighbors of each node (Method specific to track the subgoals created by effectance)."""    
    def write_header(self):
        """Write the header of the file."""
        super().write_header()
        self._write_file("Goal\tNeighbor1\tNeighbor2\n")
        self.ltm_client = ServiceClient(GetNodeFromLTM, f'{self.node.LTM_id}/get_node')
    
    def write(self):
        """Writes neighbors list."""        
        if self.node.iteration == self.node.iterations:
            response = self.ltm_client.send_request(name="")
            nodes = yaml.safe_load(response.data)
            for goal in nodes['Goal']:
                if 'reach' in goal or 'goal_' in goal:
                    self._write_file(str(goal) + "\t")
                    self._write_file(str(nodes['Goal'][goal]["neighbors"][0]["name"]) + "\t")
                    if 'reach' in goal:
                        self._write_file(str(nodes['Goal'][goal]["neighbors"][1]["name"]) + "\n")
                    else:
                        self._write_file("\n")

class FileNeighborsFull(File):
    """A file that saves the full neighbor tree at the end of an experiment."""    
    def write_header(self):
        """Write the header of the file."""
        super().write_header()
        self._write_file("Goal\tNeighbor1\tNeighbor2\n")
        self.ltm_client = ServiceClient(GetNodeFromLTM, f'{self.node.LTM_id}/get_node')
    
    def write(self):
        """Writes neighbors list for each node.""" 
        if self.node.iteration == self.node.iterations:
            response = self.ltm_client.send_request(name="")
            nodes = yaml.safe_load(response.data)
            for goal in nodes['Goal']:
                self._write_file(str(goal) + "\t")
                self._write_file(str(nodes['Goal'][goal]["neighbors"]) + "\n")

class FileEpisodesDataset(File):
    """A file that records the episodes published"""
    def __init__(self, max_size=10000, **kwargs):
        super().__init__(**kwargs)
        self.episodic_buffer = EpisodicBuffer(self.node, main_size=max_size, secondary_size=0, inputs=["old_perception", "action",
                                                                                                    "parent_policy", 
                                                                                                    "perception", "rewards"], flexible_labels=True)
        self.semaphore = threading.Semaphore()

    def write_episode(self, msg):
        self.semaphore.acquire()
        episode = container_msg_to_episode(msg)
        self.node.get_logger().debug(f"Received episode to write: {episode}")
        self.episodic_buffer.add_episode(episode)
        self.node.get_logger().debug(f"Episodic buffer size: {self.episodic_buffer.main_size}")
        self.semaphore.release()

    def write(self):
        return None
    
    def close(self):
        self.semaphore.acquire()    
        dataframe = self.episodic_buffer.get_dataframes()[0]
        if dataframe is not None:
            dataframe.to_csv(self.file_object)
        super().close()
        self.semaphore.release()

class FileDrivesPerceptions(File):
    """A file where several drive evaluation statistics about an experiment are stored."""

    def write_header(self):
        """Write the header of the file."""
        super().write_header()
        self.file_object.write(
            "Iteration\tWorld\tDrives list\tPolicy\tPerceptions\tEvaluation\n"
        )

    def write(self):
        """Write the drive evaluation and missions."""
        list_drives ={}
        drives = self.node.LTM_cache["Drive"].items()
        missions = self.node.LTM_cache["RobotPurpose"].items()
        for drive, info in drives:
            if "activation" in info:
                d_activation = info["activation"]
                for mission, n_info in missions:
                    drive_tag = drive.replace("_drive", "")
                    mission_tag = mission.replace("_mission", "")
                    if "activation" in n_info and drive_tag == mission_tag:
                        n_activation = n_info["activation"]
                        list_drives[drive] = ["evaluation: ", d_activation/n_activation if n_activation != 0 else 0.0]
        formatted = {}
        for key, value in self.node.perception_cache.items():
            data_list = value.get("data", [])
            if data_list:
                clean_data = {
                    k: v for k, v in data_list[0].items()
                }
                formatted[key] = clean_data
        self.file_object.write(
            str(self.node.iteration)
            + "\t"
            + self.node.current_world
            + "\t"
            + str(list_drives)
            + "\t"
            + self.node.current_policy
            + "\t"
            + str(formatted)
            + "\t"
            + "\n"
        )


class FileWorldModelSuccess(File):
    """A file that records the success of the world model predictions."""
    def __init__(self, ident, file_name, node, **params):
        super().__init__(ident, file_name, node, **params)
        self.subscriptions = {}

    def write_header(self):
        super().write_header()
        self._write_file(
            "Iteration\tWorld_Model\tSuccess\n"
        )

    def write(self):
        # Check world model nodes in LTM cache
        world_models = self.node.LTM_cache.get("WorldModel", {})
        for wm_name in world_models:
            if wm_name not in self.subscriptions:
                self.subscriptions[wm_name] = self.node.create_subscription(
                    SuccessRate,
                    f'world_model/{wm_name}/prediction_error',
                    self.success_rate_callback,
                    1,
                    callback_group=self.node.cbgroup_client
                )

    def success_rate_callback(self, msg):
        self._write_file(
            str(self.node.iteration)
            + "\t"
            + msg.node_name
            + "\t"
            + f"{msg.success_rate:.4f}"
            + "\n"
        )


class FileSaveModels(File):
    """A file that triggers the saving of models in nodes."""
    def __init__(self, ident, file_name, node, save_interval=100, **params):
        super().__init__(ident, file_name, node, **params)
        self.folder_path = None
        self.save_model_clients = {}
        self.save_interval = save_interval
        self.node_type_mapping = {
            "DeliberativeModel": "deliberative_model",
            "PNode": "pnode",
            "WorldModel": "world_model",
            "UtilityModel": "utility_model"
        }
        self.consecutive_index = 0

    
    def write_header(self):
        """Create the output folder."""
        i = 0
        while os.path.exists(f"{self.file_name}_{i}"):
            i = i + 1
        
        self.consecutive_index = i
        folder_path_def = f"{self.file_name}_{i}"
        os.makedirs(folder_path_def, exist_ok=True)
        self.folder_path = folder_path_def
        self.file_object = folder_path_def

    def write(self):
        # Trigger save models in all nodes in LTM cache

        write_needed = (self.save_interval > 0 and self.node.iteration % self.save_interval == 0) or (self.node.iteration == self.node.iterations)
        if write_needed:
            for node_type in self.node.LTM_cache:
                if node_type in self.node_type_mapping:
                    service_prefix = self.node_type_mapping[node_type]
                    for node_name in self.node.LTM_cache[node_type]:
                        if node_name not in self.save_model_clients:
                            self.save_model_clients[node_name] = ServiceClient(
                                SaveModel,
                                f'{service_prefix}/{node_name}/save_model'
                            )
                        self.save_model_clients[node_name].send_request(prefix=f"{self.file_name}_{self.consecutive_index}/", suffix=f"_iter_{self.node.iteration}")

    def close(self):
        return None



