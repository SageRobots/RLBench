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
from pyrep.const import ObjectType, PrimitiveShape
import numpy as np

class CubeWorld(Task):

    def init_task(self) -> None:
        self.success_sensor = ProximitySensor('success1')
        self.boundary = SpawnBoundary([Shape('plane')])
        # spawn obstacle cubes
        self.blocks = []
        for i in range(32):
            block = Shape.create(type=PrimitiveShape.CUBOID, size = [0.01, 0.01, 0.01], position=[0.0, 0.01 * i, 0.0], color=[1, 0, 0])
            self.blocks.append(block)
        # spawn end effector
        self.ee = Shape.create(type=PrimitiveShape.CUBOID, size = [0.02, 0.02, 0.01], position=[0.0, 0.0, 0.0], color=[0, 1, 0])

    def init_episode(self, index: int) -> List[str]:
        upper_left = [0.75, 0.0, 0.75]
        # generate occupancy grid
        n_occupied = np.random.randint(8, 33)
        # n_occupied = 128
        occupied = np.random.choice(256, n_occupied, replace=False)
        grid = np.zeros((256))
        grid[occupied] = 1
        grid = grid.reshape((16, 16))
        grid[7:9, 7:9] = 0
        # move all cubes to inactive positions
        for i in range(len(self.blocks)):
            self.blocks[i].set_position([upper_left[0] - 0.01 * i, 0, 0])
        # move cubes to occupied positions
        active_blocks = []
        block_i = 0
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                if grid[i, j] == 1:
                    x = upper_left[0] - 0.01 * j
                    y = upper_left[1] - 0.01 * i
                    self.blocks[block_i].set_position([x, y, upper_left[2]])
                    self.blocks[block_i].set_orientation([0, 0, 0])
                    active_blocks.append(self.blocks[block_i])
                    block_i += 1
        # move end effector to center position
        self.ee.set_position([upper_left[0] - 0.01 * 7.5, upper_left[1] - 0.01 * 7.5, upper_left[2]])
        self.ee.set_orientation([0, 0, 0])



        # n_occupied = np.random.randint(8, 33)
        # occupied = np.random.choice(256, n_occupied, replace=False)
        # active_blocks = []
        # for i_occupied in range(len(self.blocks)):
        #     if i_occupied < n_occupied:
        #         # check position
        #         x = upper_left[0] - 0.01 * (occupied[i_occupied] % 16)
        #         y = upper_left[1] - 0.01 * (occupied[i_occupied] // 16)
        #         if x < upper_left[0] - 0.01 * 7 or x > upper_left[0] + 0.01 * 9:
        #             continue
        #         self.blocks[i_occupied].set_position([x, y, upper_left[2]])
        #         self.blocks[i_occupied].set_orientation([0, 0, 0])
        #         active_blocks.append(self.blocks[i_occupied])
        #     else:
        #         self.blocks[i_occupied].set_position([upper_left[0] - 0.01 * (i_occupied % 16), upper_left[1] - 0.01 * (i_occupied // 16), 0])

        return 'n/a'

    def variation_count(self) -> int:
        return 1
    
    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        return ((0, 0, 0), (0, 0, 0))
    
    def is_static_workspace(self) -> bool:
        return True