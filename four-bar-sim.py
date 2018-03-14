from pydrake.common import FindResourceOrThrow

import os.path
from pydrake.all import (DiagramBuilder, FloatingBaseType, RigidBodyPlant,
                         RigidBodyTree, Simulator, VectorSystem, 
                         ConstantVectorSource, CompliantMaterial, 
                         CompliantContactModelParameters, DrakeVisualizer,
                         AddFlatTerrainToWorld)

from pydrake.multibody.rigid_body_tree import (FloatingBaseType, RigidBodyFrame,
    RigidBodyTree)

from pydrake.lcm import DrakeLcm
import numpy as np

rtree = RigidBodyTree(FindResourceOrThrow("drake/examples/simple_four_bar/FourBar.urdf"),
                     FloatingBaseType.kFixed)

plant = RigidBodyPlant(rtree)
builder = DiagramBuilder()
cassie = builder.AddSystem(plant)


# Setup visualizer
lcm = DrakeLcm()
visualizer = builder.AddSystem(DrakeVisualizer(tree=rtree, 
        lcm=lcm, enable_playback=True))

builder.Connect(cassie.get_output_port(0), visualizer.get_input_port(0))


# Zero inputs -- passive forward simulation
u0 = ConstantVectorSource(np.zeros(rtree.get_num_actuators()))
null_controller = builder.AddSystem(u0)

builder.Connect(null_controller.get_output_port(0), cassie.get_input_port(0))

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(True)

state = simulator.get_mutable_context().get_mutable_continuous_state_vector()

# nominal standing state
state.SetFromVector(0.01*np.random.randn(rtree.get_num_positions()*2))

simulator.StepTo(5)

