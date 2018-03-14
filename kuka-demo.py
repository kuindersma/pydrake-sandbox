import os.path
from pydrake.parsers import PackageMap
from pydrake.common import FindResourceOrThrow
from pydrake.multibody.rigid_body_tree import (
    AddModelInstanceFromUrdfStringSearchingInRosPackages,
    FloatingBaseType,
    RigidBodyFrame,
    RigidBodyTree,
)
from pydrake.lcm import DrakeLcm
from pydrake.systems.framework import BasicVector
from pydrake.systems.analysis import Simulator
from pydrake.multibody.rigid_body_plant import DrakeVisualizer
import numpy as np
import ipywidgets as widgets

def add_fixed_model(tree, filename, xyz = np.zeros(3), rpy = np.zeros(3)):
    with open(filename) as f:
        urdf_string = f.read()
    base_dir = os.path.dirname(filename)
    package_map = PackageMap()
    floating_base_type = FloatingBaseType.kFixed
    weld_frame = RigidBodyFrame("weld_frame", tree.FindBody("world"), xyz, rpy)
    AddModelInstanceFromUrdfStringSearchingInRosPackages(
        urdf_string,
        package_map,
        base_dir,
        floating_base_type,
        weld_frame,
        tree)


class DrakeVisualizerHelper:
    def __init__(self, tree):
        lcm = DrakeLcm()
        self.tree = tree
        self.visualizer = DrakeVisualizer(tree=self.tree, lcm=lcm, enable_playback=True)
        self.x = np.concatenate([robot.getZeroConfiguration(), 
                                 np.zeros(tree.get_num_velocities())])
        # Workaround for the fact that PublishLoadRobot is not public. 
        # Ultimately that should be fixed.
        sim = Simulator(self.visualizer)
        context = sim.get_mutable_context()
        context.FixInputPort(0, BasicVector(self.x))
        sim.Initialize()
    def draw(self, q = None):
        if q is not None:
            self.x[:self.tree.get_num_positions()] = q
        context = self.visualizer.CreateDefaultContext()
        context.FixInputPort(0, BasicVector(self.x))
        self.visualizer.Publish(context)
        
    def inspect(self, slider_scaling = 1):
        # Setup widgets
        for i in range(robot.number_of_positions()):
            widgets.interact(
                self.__slider_callback,
                slider_value = widgets.FloatSlider(
                    value=slider_scaling * self.x[i],
                    min=slider_scaling * self.tree.joint_limit_min[i],
                    max=slider_scaling * self.tree.joint_limit_max[i],
                    description=self.tree.get_position_name(i)
                ),
                index=widgets.fixed(i),
                slider_scaling=widgets.fixed(slider_scaling)
            )

    def __slider_callback(self, slider_value, index, slider_scaling):
        self.x[index] = slider_value / slider_scaling
        self.draw()


#iiwa_file = FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/iiwa14_spheres_collision.urdf")
iiwa_file = "/home/scottk/code/drake/share/drake/manipulation/models/iiwa_description/urdf/iiwa14_spheres_collision.urdf"

robot = RigidBodyTree()
add_fixed_model(robot, iiwa_file)
vis_helper = DrakeVisualizerHelper(robot)

q = robot.getRandomConfiguration()
vis_helper.draw(q)

