import importlib
import math
from cognitive_node_interfaces.msg import Perception, Actuation, ObjectParameters
from enum import Enum

def class_from_classname(class_name):
    """
    Return a class object from a class name.

    :param class_name: The name of the class.
    :type class_name: str
    :return: The class object.
    :rtype: type
    """
    module_string, _, class_string = class_name.rpartition(".")
    module = importlib.import_module(module_string)
    class_object = getattr(module, class_string)
    return class_object

def perception_dict_to_msg(perception_dict):
    """
    Transform a perception dictionary into a ROS message.

    :param perception_dict: Dictionary that contais the perceptions.
    :type perception_dict: dict
    :return: The ROS message with the perception.
    :rtype: cognitve_node_interfaces.msg.Perception
    """
    msg = Perception()
    if perception_dict:
        dict_to_msg(msg, perception_dict)
    else:
        msg.data = []
    return msg

def actuation_dict_to_msg(actuation_dict):
    """
    Transform an actuation dictionary into a ROS message.

    :param actuation_dict: Dictionary that contais the actuation signal.
    :type actuation_dict: dict
    :return: The ROS message with the actuation.
    :rtype: cognitve_node_interfaces.msg.Actuation
    """
    msg = Actuation()
    if actuation_dict:
        dict_to_msg(msg, actuation_dict)
    else:
        msg.data = []
    return msg

def dict_to_msg(msg, object_dict):
    """
    Transform an object dictionary into a ROS message.

    :param msg: Message to transform
    :type msg: cognitve_node_interfaces.msg.Perception or cognitve_node_interfaces.msg.Actuation
    :param object_dict: Dictionary that contais the data.
    :type object_dict: dict
    :return: The ROS message with the perception.
    :rtype: cognitve_node_interfaces.msg.Perception or cognitve_node_interfaces.msg.Actuation
    """

    if object_dict:
        msg.layout.data_offset = 0
        msg.layout.dim = []
        len_float = 8 #bytes
        for object, data in object_dict.items():
            for index, values in enumerate(data):
                dimension = ObjectParameters()
                dimension.size_stride_units = 'bytes'
                dimension.object = object + str(index)
                dimension.labels = list(values.keys())
                dimension.size = len(values)*len_float #bytes
                dimension.stride = len_float #bytes
                msg.layout.dim.append(dimension)
                for value in values.values():
                    if math.isnan(value):
                        msg.is_valid.append(False)
                        msg.data.append(0.0)
                    else:
                        msg.is_valid.append(True)
                        msg.data.append(value)
    else:
        msg.data = []
    return msg

def perception_msg_to_dict(msg):
    """
    Transform a ROS message that contains a perception into a dictionary.

    :param msg: The ROS message with the perception.
    :type msg: cognitve_node_interfaces.msg.Perception
    :return: The dictionary with the perceptions.
    :rtype: dict
    """
    perception_dict = msg_to_dict(msg)

    return perception_dict

def actuation_msg_to_dict(msg):
    """
    Transform a ROS message that contains an actuation into a dictionary.

    :param msg: The ROS message with the perception.
    :type msg: cognitve_node_interfaces.msg.Actuation
    :return: The dictionary with the actuation.
    :rtype: dict
    """
    actuation_dict= msg_to_dict(msg)
    return actuation_dict


def msg_to_dict(msg):
    """
    Transform a ROS message that contains an object list into a dictionary.

    :param msg: The ROS message containing the object list.
    :type msg: cognitve_node_interfaces.msg.Perception or cognitve_node_interfaces.msg.Actuation
    :return: A dictionary representation of the object list.
    :rtype: dict
    """
    dict = {}
    first_value = 0 
    for dim in msg.layout.dim:
        object = dim.object[:-1]
        labels = dim.labels
        size = dim.size
        stride = dim.stride
        num_elements = size//stride
        final_value = num_elements + first_value
        values = msg.data[first_value:final_value]
        flags = msg.is_valid[first_value:final_value]
        values_dict = {labels[i]: values[i] if flags[i] else float('nan') for i in range(len(labels))}

        if not object in dict.keys():
            dict[object] = [values_dict]
        else:
            dict[object].append(values_dict)
            
        first_value += num_elements

    return dict


def separate_perceptions(perception):
    """
    Separate a dicionary with several perceptions in several ones with one perception.

    :param perception: The dictionary with all perceptions.
    :type perception: dict
    :return: A list with the dictionaries.
    :rtype: list
    """
    perceptions = []
    for i in range(max([len(sensor) for sensor in perception.values()])):
            perception_line = {} 
            for sensor, value in perception.items():
                sid = i % len(value)
                perception_line[sensor + str(sid)] = value[sid]
            perceptions.append(perception_line)

    return perceptions

def compare_perceptions(input_1, input_2, thresh=0.01):
    """
    Return True if both perceptions have the same value. False otherwise.

    :param sensing: Sensing in the current iteration.
    :type sensing: dict
    :param old_sensing: Sensing in the last iteration.
    :type old_sensing: dict
    :return: Boolean that indicates if there is a sensorial change.
    :rtype: bool
    """

    for sensor in input_1:
        for perception_1, perception_2 in zip(input_1[sensor], input_2[sensor]):
            if isinstance(perception_1, dict):
                for attribute in perception_1:
                    difference = abs(perception_1[attribute] - perception_2[attribute])
                    if difference > thresh:
                        return False
            else:
                if abs(perception_1[0] - perception_2[0]) > thresh:
                    return False
    return True

class EncodableDecodableEnum(Enum):
    """Enum class that can be encoded and decoded to/from a normalized value."""

    @classmethod
    def encode(cls, value: str, normalized=True) -> float:
        """
        Encodes a string to a normalized class value.

        :param value: The string representation of the enum member.
        :type value: str
        :param normalized: Whether to normalize the encoded value, defaults to True.
        :type normalized: bool
        :raises ValueError: If the provided value is not a valid enum member.
        :return: The encoded value as a float
        :rtype: float
        """
        value = value.upper().replace(" ", "_")
        if value not in cls.__members__:
            raise ValueError(f"{value} is not a valid member of {cls.__name__}")
        if normalized:
            return cls[value].value / (len(cls) - 1)
        else:
            return cls[value].value
    @classmethod
    def decode(cls, value, normalized=True) -> str:
        """
        Decodes a normalized class value back to the corresponding string.

        :param value: The encoded value to decode.
        :type value: float
        :param normalized: Whether the value is normalized, defaults to True.
        :type normalized: bool
        :raises ValueError: If no matching class is found for the value.
        :return: The string representation of the enum member.
        :rtype: str
        """
        if normalized:
            index = round(value * (len(cls) - 1))
        else:
            index = int(value)
        for member in cls:
            if member.value == index:
                return member.name
        raise ValueError(f"No matching class for normalized value {value}")