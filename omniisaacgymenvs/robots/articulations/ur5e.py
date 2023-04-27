
from typing import Optional
import math
import numpy as np
import torch
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.utils.nucleus import get_server_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omniisaacgymenvs.tasks.utils.usd_utils import set_drive

from omni.isaac.core.utils.prims import get_prim_at_path
from pxr import PhysxSchema


class UR5e(Robot):
    def __init__(
        self,
        prim_path: str,
        name: Optional[str] = "ur5e",
        usd_path: Optional[str] = None,
        translation: Optional[torch.tensor] = None,
        orientation: Optional[torch.tensor] = None,
    ) -> None:
        """[summary]
        """

        self._usd_path = usd_path
        self._name = name

        self._position = torch.tensor([1.0, 0.0, 0.0]) if translation is None else translation
        self._orientation = torch.tensor([0.0, 0.0, 0.0, 1.0]) if orientation is None else orientation

        if self._usd_path is None:
            assets_root_path = get_server_path()
            if assets_root_path is None:
                carb.log_error("Could not find root assets folder")
            self._usd_path = assets_root_path + "/Users/cambel/ur5e/ur5e_instanceable.usd"

        add_reference_to_stage(self._usd_path, prim_path)

        super().__init__(
            prim_path=prim_path,
            name=name,
            translation=self._position,
            orientation=self._orientation,
            articulation_controller=None,
        )

        dof_paths = [
            "base_link_inertia/shoulder_pan_joint",
            "shoulder_link/shoulder_lift_joint",
            "upper_arm_link/elbow_joint",
            "forearm_link/wrist_1_joint",
            "wrist_1_link/wrist_2_joint",
            "wrist_2_link/wrist_3_joint",
        ]

        drive_type = ["angular"] * 6
        default_dof_pos = [math.degrees(x) for x in [1.56, -1.37, 1.64, -1.84, -1.56, 0.0]]
        stiffness = [400*np.pi/180] * 6
        damping = [80*np.pi/180] * 6
        max_force = [150, 150, 150, 28, 28, 28]
        max_velocity = [180., 180., 180., 180., 180., 180.]

        for i, dof in enumerate(dof_paths):
            set_drive(
                prim_path=f"{self.prim_path}/{dof}",
                drive_type=drive_type[i],
                target_type="position",
                target_value=default_dof_pos[i],
                stiffness=stiffness[i],
                damping=damping[i],
                max_force=max_force[i]
            )

            PhysxSchema.PhysxJointAPI(get_prim_at_path(f"{self.prim_path}/{dof}")).CreateMaxJointVelocityAttr().Set(max_velocity[i])
