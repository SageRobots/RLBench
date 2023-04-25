from typing import List, Tuple
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.spawn_boundary import SpawnBoundary
from pyrep.objects.dummy import Dummy
from rlbench.const import colors
import numpy as np
import time

class PushBlock(Task):

    def init_task(self) -> None:
        self.success_sensor = ProximitySensor('success1')
        self.boundary = SpawnBoundary([Shape('plane')])
        # blocks
        self.block1 = Shape('block1')
        self.block2 = Shape('block2')
        # waypoints
        self.waypoint0 = Dummy('waypoint0')
        self.waypoint1 = Dummy('waypoint1')
        self.waypoint2 = Dummy('waypoint2')
        # bowl
        self.register_success_conditions([
            DetectedCondition(self.block1, self.success_sensor)
        ])
    
    def adjust_waypoints(self, block1_pos):
        # move waypoint 0 behind block 1
        self.waypoint0.set_position([-0.1, 0.0, 0.1], relative_to=self.block1)
        # move waypoint 1 behind block 1
        self.waypoint1.set_position([-0.1, 0.0, 0.02], relative_to=self.block1)
        # move waypoint 2 inside block 1
        self.waypoint2.set_position([0.04, 0.0, 0.02], relative_to=self.block1)
        # move the sensor in front of block 1
        self.success_sensor.set_position([block1_pos[0]+0.08, block1_pos[1], block1_pos[2]-0.04])

    def init_episode(self, index: int) -> List[str]:
        # move robot to initial position
        j = np.array([1.90242633e-01, -1.82561681e-03, -1.74581066e-01, -2.33221745e+00, -1.09314790e-03,  2.26251936e+00,  8.01950991e-01])
        self.robot.arm.set_joint_positions(j, disable_dynamics=True)

        # spawn objects in the workspace
        self.boundary.clear()
        for ob in [self.block1, self.block2]:
            self.boundary.sample(ob, ignore_collisions=False, min_distance=0.16, 
                                 min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
        
        # get the position of block1
        block1_pos = self.block1.get_position()

        # set the block colors
        text = []
        valid_idx = np.array([2, 6, 7, 8])
        if index == 0:
            color = 8
            self.block1.set_color(colors[color][1])
            text.append('push the magenta block left')
            # move the sensor next to the block
            self.success_sensor.set_position([block1_pos[0], block1_pos[1]-0.08, block1_pos[2]-0.04])
        elif index == 1:
            color = 7
            self.block1.set_color(colors[color][1])
            text.append('push the cyan block forward')
            self.adjust_waypoints(block1_pos)
        elif index == 2:
            color = 8
            # push magenta block forward
            self.block1.set_color(colors[color][1])
            text.append('push the magenta block forward')
            self.adjust_waypoints(block1_pos)
        elif index == 3:
            color = 7
            # push cyan block left
            self.block1.set_color(colors[color][1])
            text.append('push the cyan block left')
            self.success_sensor.set_position([block1_pos[0], block1_pos[1]-0.08, block1_pos[2]-0.04])
        elif index == 4:
            color = 6
            # push yellow block left
            self.block1.set_color(colors[color][1])
            text.append('push the yellow block left')
            self.success_sensor.set_position([block1_pos[0], block1_pos[1]-0.08, block1_pos[2]-0.04])
        elif index == 5:
            # push yellow block forward
            color = 6
            self.block1.set_color(colors[color][1])
            text.append('push the yellow block forward')
            self.adjust_waypoints(block1_pos)
        elif index == 6:
            # push lime block left
            color = 2
            self.block1.set_color(colors[color][1])
            text.append('push the lime block left')
            self.success_sensor.set_position([block1_pos[0], block1_pos[1]-0.08, block1_pos[2]-0.04])
        elif index == 7:
            # push lime block forward
            color = 2
            self.block1.set_color(colors[color][1])
            text.append('push the lime block forward')
            self.adjust_waypoints(block1_pos)

        # random valid color for distractor block
        color2 = np.random.choice(valid_idx)
        while color2 == color:
            color2 = np.random.choice(valid_idx)
        self.block2.set_color(colors[color2][1])

        return text

    def variation_count(self) -> int:
        return 8
    
    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        return ((0, 0, 0), (0, 0, 0))
    
    def is_static_workspace(self) -> bool:
        return True