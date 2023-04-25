from typing import List, Tuple
import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.const import colors
from rlbench.backend.task import Task
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.conditions import DetectedCondition
import time

class ReachTargetLang(Task):

    def init_task(self) -> None:
        self.target = Shape('target')
        self.distractor1 = Shape('distractor1')
        self.distractor2 = Shape('distractor2')
        self.boundary1 = SpawnBoundary([Shape('boundary1')])
        self.boundary2 = SpawnBoundary([Shape('boundary2')])
        self.boundary3 = SpawnBoundary([Shape('boundary3')])
        self.boundary4 = SpawnBoundary([Shape('boundary4')])
        success_sensor = ProximitySensor('success')
        self.register_success_conditions(
            [DetectedCondition(self.robot.arm.get_tip(), success_sensor)])

    def init_episode(self, index: int) -> List[str]:
        # move robot in plane with boundary
        j = np.array([-19.98446757,  -5.13051495,  19.61395883, -93.61771465, 1.72630822, 85.01589052, 44.61888011])*np.pi/180
        self.robot.arm.set_joint_positions(j, disable_dynamics=True)
        

        if index == 0:
            # red target
            self.target.set_color(colors[0][1])
            # green distractor
            self.distractor1.set_color(colors[3][1])
            # blue distractor
            self.distractor2.set_color(colors[4][1])
            # boundary1 red
            b = self.boundary1
            b.clear()
            b.sample(self.target, min_distance=0.2,
                     min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
            # boundary2 green
            b = self.boundary2
            b.clear()
            b.sample(self.distractor1, min_distance=0.2,
                        min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
            # boundary3 blue
            b = self.boundary3
            b.clear()
            b.sample(self.distractor2, min_distance=0.2,
                        min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
            return ['reach the red target']
        elif index == 1:
            # green target
            self.target.set_color(colors[3][1])
            # red distractor
            self.distractor1.set_color(colors[0][1])
            # blue distractor
            self.distractor2.set_color(colors[4][1])
            # boundary1 red
            b = self.boundary1
            b.clear()
            b.sample(self.distractor1, min_distance=0.2,
                        min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
            # boundary2 green
            b = self.boundary2
            b.clear()
            b.sample(self.target, min_distance=0.2,
                     min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
            # boundary3 blue
            b = self.boundary3
            b.clear()
            b.sample(self.distractor2, min_distance=0.2,
                        min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
            return ['reach the green target']
        elif index == 2:
            # blue target
            self.target.set_color(colors[4][1])
            # red distractor
            self.distractor1.set_color(colors[0][1])
            # green distractor
            self.distractor2.set_color(colors[3][1])
            # boundary1 red
            b = self.boundary1
            b.clear()
            b.sample(self.distractor1, min_distance=0.2,
                        min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
            # boundary2 green
            b = self.boundary2
            b.clear()
            b.sample(self.distractor2, min_distance=0.2,
                        min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
            # boundary3 blue
            b = self.boundary3
            b.clear()
            b.sample(self.target, min_distance=0.2,
                     min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
            return ['reach the blue target']
        elif index == 3:
            # boundary4
            b = self.boundary4
            b.clear()
            b.sample(self.target, min_distance=0.2,
                     min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
            
        return ['reach the target']

    def variation_count(self) -> int:
        return 3

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

    def get_low_dim_state(self) -> np.ndarray:
        # One of the few tasks that have a custom low_dim_state function.
        return np.array(self.target.get_position())

    def is_static_workspace(self) -> bool:
        return True

    def reward(self) -> float:
        return -np.linalg.norm(self.target.get_position() -
                               self.robot.arm.get_tip().get_position())
