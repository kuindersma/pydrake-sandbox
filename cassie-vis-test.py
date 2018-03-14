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
  



robot = RigidBodyTree("cassie/urdf/cassie-simple.urdf",
                     FloatingBaseType.kQuaternion)

vis_helper = DrakeVisualizerHelper(robot)

q = robot.getRandomConfiguration()
vis_helper.draw(q)
