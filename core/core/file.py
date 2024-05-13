from math import isclose
import yaml
import yamlloader


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
            "Iteration\tGoal\tWorld\tReward\tPolicy\tSensorial changes\tC-nodes\n"
        )

    def write(self):
        """Write statistics data."""
        self.file_object.write(
            str(self.node.iteration)
            + "\t"
            + (self.node.current_goal if self.node.current_goal else "None")
            + "\t"
            + self.node.current_world
            + "\t"
            + str(f"{self.node.current_reward:.1f}")
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