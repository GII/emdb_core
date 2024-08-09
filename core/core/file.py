from rclpy.time import Time

from cognitive_node_interfaces.msg import Activation
from cognitive_processes_interfaces.msg import Episode
from core.utils import perception_msg_to_dict

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
        self.file_object = open(self.file_name, "a", encoding="utf-8")

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
            + "\t\t"
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


class FileNodeActivations(File):
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.node_subscriptions = []
        self.write_header()
        self.check_subscriptions()

    def write_header(self):
        """Write the header of the file."""
        super().write_header()
        self.file_object.write(
            "Timestamp\tNode name\tNode type\tActivation\n"
        )

    def check_subscriptions(self):
        for node in self.node.LTM_cache:
            if node['name'] not in self.node_subscriptions:
                subscriber = self.node.create_subscription(
                Activation,
                'cognitive_node/' + str(node['name']) + '/activation',
                self.receive_activation_callback,
                1,
                callback_group=self.node.cbgroup_loop
                )
                self.node_subscriptions.append(node['name'])

    def receive_activation_callback(self, msg: Activation):
        if not self.file_object.closed:
            timestamp = Time.from_msg(msg.timestamp).seconds_nanoseconds()

            self.file_object.write(
                f"{(timestamp[0] + timestamp[1]*1e-9):.3f}"
                + "\t"
                + msg.node_name
                + "\t"
                + msg.node_type
                + "\t"
                + f"{msg.activation:.2f}"
                + "\n"
            )

    def write(self):
        self.check_subscriptions()

# WORK IN PROGRESS

# class FilesEpisodes(File):
#     def __init__(self, **kwargs):
#         super().__init__(**kwargs)
#         self.write_header()
#         self.node.create_subscription(
#             Episode,
#             'main_loop/episodes',
#             self.receive_episodes_callback,
#             1,
#             callback_group = self.node.cbgroup_loop
#         )
    
#     def write_header(self):
#         """Write the header of the file."""
#         super().write_header()
#         self.file_object.write(
#             "Timestamp\tOld perception\tOld LTM state\tPolicy\tPerception\tLTM state\tReward list\n"
#         )
    
#     def receive_episodes_callback(self, msg: Episode):
#         self.file_object.write(
#             str(Time.from_msg(msg.timestamp).nanoseconds)
#             + "\t"
#             + str(perception_msg_to_dict(msg.old_perception))
#             #+ "\t"
#             #+ msg.old_ltm_state
#             + "\t"
#             + msg.policy
#             + "\t"
#             + str(perception_msg_to_dict(msg.perception))
#             #+ "\t"
#             #+ msg.ltm_state
#             + "\t"
#             + msg.reward_list
#             + "\n"
#         )

#     def write(self):
#         pass



