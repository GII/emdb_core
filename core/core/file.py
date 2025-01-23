from rclpy.time import Time
import os
import yaml

from core.service_client import ServiceClient
from cognitive_node_interfaces.msg import Activation
from cognitive_node_interfaces.srv import SendSpace
from cognitive_processes_interfaces.msg import Episode
from core_interfaces.srv import GetNodeFromLTM
from core.utils import perception_msg_to_dict, separate_perceptions

class File():
    """A MDB file."""

    def __init__(self, **kwargs):
        """Init attributes when a new object is created."""
        self.ident = kwargs["ident"]
        self.file_name = kwargs["file_name"]
        self.file_object = None
        self.data = kwargs.get("data")
        self.node = kwargs["node"]

    def __getstate__(self):
        """Return the object to be serialize with PyYAML as the result of removing the unpicklable entries."""
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

class FileGoodness(File):
    """A file where several goodness statistics about an experiment are stored."""

    def write_header(self):
        """Write the header of the file."""
        super().write_header()
        self.file_object.write(
            "Iteration\tWorld\tGoal reward list\tPolicy\tSensorial changes\tC-nodes\n"
        )

    def write(self):
        """Write statistics data."""
        formatted_goals = {goal: f"{reward:.1f}" for goal, reward in self.node.stm.reward_list.items()}

        self.file_object.write(
            str(self.node.iteration)
            + "\t"
            + self.node.current_world
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
    """A file that records wether a P-node's activation has been successful or not."""

    def write_header(self):
        """Write the header of the file."""
        super().write_header()
        self.file_object.write("Iteration\tIdent\tSuccess\n")

    def write(self):
        """Write success."""
        for pnode, success in self.node.pnodes_success.items():
            if success is not None:
                self.file_object.write(
                    str(self.node.iteration)
                    + "\t"
                    + pnode
                    + "\t"
                    + str(success)
                    + "\n"
                )
            self.node.pnodes_success[pnode] = None

class FileTrialsSuccess(File):
    """A file that records wether a P-node's activation has been successful or not."""

    def write_header(self):
        """Write the header of the file."""
        super().write_header()
        self.file_object.write("Iteration\tTrial\tIterations\tSuccess\n")

    def write(self):
        """Write success."""
        for iteration, trial, iterations, success in self.node.trials_data:
            self.file_object.write(
                str(iteration)
                + "\t"
                + str(trial)
                + "\t"
                + str(iterations)
                + "\t"
                + str(success)
                + "\n"
            )
        self.node.trials_data = []

class FilePNodesContent(File):
    def write_header(self):
        super().write_header()
        self.file_object.write("Iteration\tIdent\t")
        self.header_finished = False
        self.created_clients = {}
        self.ite = 100

    def create_pnode_client(self, pnode_name):
        if pnode_name not in self.created_clients:
            pnode_client = ServiceClient(SendSpace, 'pnode/' + str(pnode_name) + '/send_space')
            self.created_clients[pnode_name] = pnode_client

    def finish_header(self, labels):
        for label in labels:
            self.file_object.write(f"{label}\t")
        self.file_object.write("Confidence\n")
        self.header_finished = True
        self.labels = labels

    def write(self):
        if "PNode" in self.node.LTM_cache and self.node.iteration % 100 == 0:
            for pnode in self.node.LTM_cache["PNode"]:
                if pnode not in self.created_clients:
                    self.create_pnode_client(pnode)

                if self.created_clients[pnode]:
                    response = self.created_clients[pnode].send_request()

                    labels = response.labels

                    if labels:
                        if not self.header_finished:
                            self.finish_header(labels)
                        
                        if labels == self.labels:
                            data = response.data
                            confidences = response.confidences

                            j = 0
                            for confidence in confidences:
                                self.file_object.write(str(self.ite) + "\t")
                                self.file_object.write(pnode + "\t")

                                for i in range(j, len(labels)+j):
                                    self.file_object.write(str(data[i]) + "\t")
                                self.file_object.write(str(confidence) + "\n")
                                j = j + len(labels)

                        else:
                            self.file_object.write("ERROR. LABELS DO NOT MATCH BETWEEN PNODES\n")
                    
                
                
                
            self.ite = self.ite + 100

class FileLastIterationPNodesContent(FilePNodesContent):
    def write(self):
        if "PNode" in self.node.LTM_cache and self.node.iteration == self.node.iterations:
            for pnode in self.node.LTM_cache["PNode"]:
                self.create_pnode_client(pnode)

                if self.created_clients[pnode]:
                    response = self.created_clients[pnode].send_request()

                    labels = response.labels

                    if labels:
                        if not self.header_finished:
                            self.finish_header(labels)
                        
                        if labels == self.labels:
                            data = response.data
                            confidences = response.confidences

                            j = 0
                            for confidence in confidences:
                                self.file_object.write(str(self.node.iterations) + "\t")
                                self.file_object.write(pnode + "\t")

                                for i in range(j, len(labels)+j):
                                    self.file_object.write(str(data[i]) + "\t")
                                self.file_object.write(str(confidence) + "\n")
                                j = j + len(labels)

                        else:
                            self.file_object.write("ERROR. LABELS DO NOT MATCH BETWEEN PNODES\n")
                    
                    
        
class FileGoalsContent(File):
    def write_header(self):
        super().write_header()
        self.file_object.write("Iteration\tIdent\t")
        self.header_finished = False
        self.created_clients = {}
        self.ite = 100

    def create_goal_client(self, goal_name):
        if goal_name not in self.created_clients:
            goal_client = ServiceClient(SendSpace, 'goal/' + str(goal_name) + '/send_space')
            self.created_clients[goal_name] = goal_client

    def finish_header(self, labels):
        for label in labels:
            self.file_object.write(f"{label}\t")
        self.file_object.write("Confidence\n")
        self.header_finished = True
        self.labels = labels

    def write(self):
        if "Goal" in self.node.LTM_cache and self.node.iteration % 100 == 0:
            for goal in self.node.LTM_cache["Goal"]:
                if goal not in self.created_clients:
                    self.create_goal_client(goal)
                if self.created_clients[goal]:
                    response = self.created_clients[goal].send_request()
                    self.node.get_logger().info(f"Writing data for goal {goal}. Points: {len(response.confidences)}")
                    labels = response.labels

                    if labels:
                        if not self.header_finished:
                            self.finish_header(labels)
                        
                        if labels == self.labels:
                            data = response.data
                            confidences = response.confidences

                            j = 0
                            for confidence in confidences:
                                self.file_object.write(str(self.ite) + "\t")
                                self.file_object.write(goal + "\t")

                                for i in range(j, len(labels)+j):
                                    self.file_object.write(str(data[i]) + "\t")
                                self.file_object.write(str(confidence) + "\n")
                                j = j + len(labels)

                        else:
                            self.file_object.write("ERROR. LABELS DO NOT MATCH BETWEEN GOALS\n")
                    
                
            self.ite = self.ite + 100
    

class FileLastIterationGoalsContent(FileGoalsContent):
    def write(self):
        if "Goal" in self.node.LTM_cache and self.node.iteration == self.node.iterations:
            for goal in self.node.LTM_cache["Goal"]:
                self.create_goal_client(goal)
                if self.created_clients[goal]:
                    response = self.created_clients[goal].send_request()
                    labels = response.labels

                    if labels:
                        if not self.header_finished:
                            self.finish_header(labels)
                        
                        if labels == self.labels:
                            data = response.data
                            confidences = response.confidences

                            j = 0
                            for confidence in confidences:
                                self.file_object.write(str(self.node.iterations) + "\t")
                                self.file_object.write(goal + "\t")

                                for i in range(j, len(labels)+j):
                                    self.file_object.write(str(data[i]) + "\t")
                                self.file_object.write(str(confidence) + "\n")
                                j = j + len(labels)

                        else:
                            self.file_object.write("ERROR. LABELS DO NOT MATCH BETWEEN GOALS\n")
                    
                    else:
                        self.created_clients[goal] = None

class FileNeighbors(File):
    def write_header(self):
        super().write_header()
        self.file_object.write("Goal\tNeighbor1\tNeighbor2\n")
        self.ltm_client = ServiceClient(GetNodeFromLTM, f'{self.node.LTM_id}/get_node')
    
    def write(self):
        if self.node.iteration == self.node.iterations:
            response = self.ltm_client.send_request(name="")
            nodes = yaml.safe_load(response.data)
            for goal in nodes['Goal']:
                if 'reach' in goal or 'goal_' in goal:
                    self.file_object.write(str(goal) + "\t")
                    self.file_object.write(str(nodes['Goal'][goal]["neighbors"][0]["name"]) + "\t")
                    if 'reach' in goal:
                        self.file_object.write(str(nodes['Goal'][goal]["neighbors"][1]["name"]) + "\n")
                    else:
                        self.file_object.write("\n")

class FileNeighborsFull(File):
    def write_header(self):
        super().write_header()
        self.file_object.write("Goal\tNeighbor1\tNeighbor2\n")
        self.ltm_client = ServiceClient(GetNodeFromLTM, f'{self.node.LTM_id}/get_node')
    
    def write(self):
        if self.node.iteration == self.node.iterations:
            response = self.ltm_client.send_request(name="")
            nodes = yaml.safe_load(response.data)
            for goal in nodes['Goal']:
                self.file_object.write(str(goal) + "\t")
                self.file_object.write(str(nodes['Goal'][goal]["neighbors"]) + "\n")




