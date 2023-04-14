from typing import List, Tuple
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.spawn_boundary import SpawnBoundary
from pyrep.objects.dummy import Dummy
import numpy as np

class KnockEraser(Task):

    def init_task(self) -> None:
        self.success_sensor = ProximitySensor('success')
        self.boundary = SpawnBoundary([Shape('plane')])
        # lemon
        self.eraser = Shape('eraser')
        # bowl
        self.register_graspable_objects([self.eraser])
        self.register_success_conditions([
            DetectedCondition(self.eraser, self.success_sensor)
        ])

    def init_episode(self, index: int) -> List[str]:
        # move robot to initial position
        j = np.array([-19.98446757,  -5.13051495,  19.61395883, -93.61771465, 1.72630822, 85.01589052, 44.61888011])*np.pi/180
        self.robot.arm.set_joint_positions(j, disable_dynamics=True)

        # spawn objects in the workspace
        self.boundary.clear()
        for ob in [self.eraser]:
            self.boundary.sample(ob, ignore_collisions=False, min_distance=0.25, 
                                 min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
            
        # get the position of the eraser
        eraser_pos = self.eraser.get_position()
        # move the eraser next to the eraser
        self.success_sensor.set_position([eraser_pos[0], eraser_pos[1]-0.07, eraser_pos[2]-0.07])
        
        return ['knock the eraser over']

    def variation_count(self) -> int:
        return 1
    
    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        return ((0, 0, 0), (0, 0, 0))
    
    def is_static_workspace(self) -> bool:
        return True