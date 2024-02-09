# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import string
import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.node import Node
import secrets


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """This function generates a test description for a launch file.
    Parameters:
        - None
    Returns:
        - launch.LaunchDescription: The test description for the launch file.
        - dict: A dictionary containing the names of the nodes to be tested.
    Processing Logic:
        - Create an empty list for launch actions.
        - Create an empty list for node names.
        - For each number in the range of 3:
            - Append a launch_ros.actions.Node object to the launch actions list with the given executable, package, and name.
            - Append the name of the node to the node names list.
        - Append a launch_testing.actions.ReadyToTest object to the launch actions list.
        - Return the launch actions list and the node names dictionary."""
    
    launch_actions = []
    node_names = []

    for i in range(3):
        launch_actions.append(
            launch_ros.actions.Node(
                executable='talker',
                package='demo_nodes_cpp',
                name='demo_node_' + str(i)
            )
        )
        node_names.append('demo_node_' + str(i))

    launch_actions.append(launch_testing.actions.ReadyToTest())
    return launch.LaunchDescription(launch_actions), {'node_list': node_names}


class CheckMultipleNodesLaunched(unittest.TestCase):

    def test_nodes_successful(self, node_list):
        """Check if all the nodes were launched correctly."""
        # Method 1
        wait_for_nodes_1 = WaitForNodes(node_list, timeout=10.0)
        assert wait_for_nodes_1.wait()
        assert wait_for_nodes_1.get_nodes_not_found() == set()
        wait_for_nodes_1.shutdown()

        # Method 2
        with WaitForNodes(node_list, timeout=10.0) as wait_for_nodes_2:
            print('All nodes were found !')
            assert wait_for_nodes_2.get_nodes_not_found() == set()

    def test_node_does_not_exist(self, node_list):
        """Insert a invalid node name that should not exist."""
        invalid_node_list = node_list + ['invalid_node']

        # Method 1
        wait_for_nodes_1 = WaitForNodes(invalid_node_list, timeout=10.0)
        assert not wait_for_nodes_1.wait()
        assert wait_for_nodes_1.get_nodes_not_found() == {'invalid_node'}
        wait_for_nodes_1.shutdown()

        # Method 2
        with pytest.raises(RuntimeError):
            with WaitForNodes(invalid_node_list, timeout=10.0):
                pass


# TODO (adityapande-1995): Move WaitForNodes implementation to launch_testing_ros
# after https://github.com/ros2/rclpy/issues/831 is resolved
class WaitForNodes:
    """
    Wait to discover supplied nodes.

    Example usage:
    --------------
    # Method 1, calling wait() and shutdown() manually
    def method_1():
        node_list = ['foo', 'bar']
        wait_for_nodes = WaitForNodes(node_list, timeout=5.0)
        assert wait_for_nodes.wait()
        print('Nodes found!')
        assert wait_for_nodes.get_nodes_not_found() == set()
        wait_for_nodes.shutdown()

    # Method 2, using the 'with' keyword
    def method_2():
        with WaitForNodes(['foo', 'bar'], timeout=5.0) as wait_for_nodes:
            assert wait_for_nodes.get_nodes_not_found() == set()
            print('Nodes found!')
    """

    def __init__(self, node_names, timeout=5.0):
        """Initializes the class object with a list of node names and a timeout value.
        Parameters:
            - node_names (list): List of node names to be checked.
            - timeout (float): Timeout value in seconds. Default is 5.0 seconds.
        Returns:
            - None: This function does not return any value.
        Processing Logic:
            - Initializes the class object.
            - Sets the list of node names.
            - Sets the timeout value.
            - Initializes the ROS context.
            - Prepares the node.
            - Sets the expected nodes as a set.
            - Initializes the nodes found variable."""
        
        self.node_names = node_names
        self.timeout = timeout
        self.__ros_context = rclpy.Context()
        rclpy.init(context=self.__ros_context)
        self._prepare_node()

        self.__expected_nodes_set = set(node_names)
        self.__nodes_found = None

    def _prepare_node(self):
        """Creates a unique node name and initializes a ROS node.
        Parameters:
            - self (object): The object itself.
        Returns:
            - ros_node (Node): The initialized ROS node.
        Processing Logic:
            - Create unique node name.
            - Initialize ROS node.
            - Use secrets.SystemRandom() to generate random characters.
            - Limit the length of the random characters to 10.
        Example:
            _prepare_node() # Creates a unique node name and initializes a ROS node."""
        
        self.__node_name = '_test_node_' +\
            ''.join(secrets.SystemRandom().choices(string.ascii_uppercase + string.digits, k=10))
        self.__ros_node = Node(node_name=self.__node_name, context=self.__ros_context)

    def wait(self):
        """Function to wait for nodes to become available.
        Parameters:
            - self (object): The object containing the function.
            - timeout (int): The maximum time to wait for nodes to become available.
        Returns:
            - flag (bool): True if all nodes are found within the timeout period, False otherwise.
        Processing Logic:
            - Starts a timer to track the timeout period.
            - Sets a flag to False to indicate that nodes have not been found yet.
            - Prints a message to indicate that the function is waiting for nodes.
            - Checks if the current time minus the start time is less than the timeout and if the flag is still False.
            - If both conditions are met, checks if all the node names provided are in the list of current node names.
            - If all node names are found, sets the flag to True.
            - Waits for 0.3 seconds before checking again.
            - Once the loop ends, sets the nodes found to a set of all current node names except for the current node.
            - Returns the flag indicating if all nodes were found within the timeout period.
        Example:
            wait(self, 10)  # Waits for 10 seconds for nodes to become available."""
        
        start = time.time()
        flag = False
        print('Waiting for nodes')
        while time.time() - start < self.timeout and not flag:
            flag = all(name in self.__ros_node.get_node_names() for name in self.node_names)
            time.sleep(0.3)

        self.__nodes_found = set(self.__ros_node.get_node_names())
        self.__nodes_found.remove(self.__node_name)
        return flag

    def shutdown(self):
        """Shuts down the ROS node and context.
        Parameters:
            - self (object): The ROS node to be shut down.
        Returns:
            - None: No return value.
        Processing Logic:
            - Destroys the ROS node.
            - Shuts down the ROS context."""
        
        self.__ros_node.destroy_node()
        rclpy.shutdown(context=self.__ros_context)

    def __enter__(self):
        """"Returns the object itself and raises a RuntimeError if all nodes are not found.
        Parameters:
            - self (object): The object itself.
        Returns:
            - object: The object itself.
        Processing Logic:
            - Raises RuntimeError if nodes are not found.
            - Returns the object itself.
            - Checks for node availability using wait() method.
            - Uses the 'not' keyword to negate the result of wait() method.""""
        
        if not self.wait():
            raise RuntimeError('Did not find all nodes !')

        return self

    def __exit__(self, exep_type, exep_value, trace):
        """Closes the connection to the server.
        Parameters:
            - exep_type (type): The type of exception that occurred.
            - exep_value (type): The value of the exception that occurred.
            - trace (type): The traceback of the exception that occurred.
        Returns:
            - None: No return value.
        Processing Logic:
            - If an exception occurred, raise an Exception with the value of the exception.
            - Call the shutdown() function.
        Example:
            __exit__(None, None, None) # Closes the connection to the server."""
        
        if exep_type is not None:
            raise Exception('Exception occured, value: ', exep_value)
        self.shutdown()

    def get_nodes_found(self):
        """"Returns the list of nodes found during a search."
        Parameters:
            - self (object): Instance of the class.
        Returns:
            - list: List of nodes found during the search.
        Processing Logic:
            - Get the list of nodes found.
            - Returns the list.
            - Private attribute is accessed using __.
            - Only accessible within the class."""
        
        return self.__nodes_found

    def get_nodes_not_found(self):
        """_set
        "Returns a set of nodes that were expected but not found in the provided data set.
        Parameters:
            - self (object): The current object.
        Returns:
            - set: A set of nodes that were expected but not found.
        Processing Logic:
            - Subtract expected nodes from found nodes.
            - Return the resulting set.
            - No error handling needed.""""
        
        return self.__expected_nodes_set - self.__nodes_found
