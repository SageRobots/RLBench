from typing import List, Tuple
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.spawn_boundary import SpawnBoundary
from pyrep.objects.dummy import Dummy

class ReachTutorial1(Task):

    def init_task(self) -> None:
        self.success_sensor = ProximitySensor('success')
        self.boundary = SpawnBoundary([Shape('plane')])
        # lemon
        self.lemon = Shape('lemon')
        self.waypoint0 = Dummy('waypoint0')
        # bowl
        self.bowl = Shape('ceramic_bowl')
        self.register_graspable_objects([self.lemon])
        self.register_success_conditions([
            DetectedCondition(self.lemon, self.success_sensor)
        ])

    def init_episode(self, index: int) -> List[str]:
        self.boundary.clear()
        # reset the lemon and bowl
        # self.lemon.set_position([1.5, 1.5, 1])
        # self.bowl.set_position([1.5, 1.5, 1])
        # spawn objects in the workspace
        # for ob in [self.lemon, self.bowl]:
        #     self.boundary.sample(ob, ignore_collisions=False, min_distance=0.1, 
        #                          min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
        # spawn the bowl in the workspace
        self.boundary.sample(self.bowl, ignore_collisions=False, min_distance=0.25, 
                             min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
        # spawn the lemon in the workspace
        self.boundary.sample(self.lemon, ignore_collisions=False, min_distance=0.25, 
                             min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
        
        # move waypoint0 above lemon
        # self.waypoint0.set_position([0, 0, 0.1], relative_to=self.lemon)
        return ['place the lemon in the bowl']

    def variation_count(self) -> int:
        return 1
    
    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        return ((0, 0, 0), (0, 0, 0))
    
    def is_static_workspace(self) -> bool:
        return True