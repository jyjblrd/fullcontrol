from typing import Optional
from pydantic import BaseModel
from fullcontrol.common import Point


class GcodeControls(BaseModel):
    'control to adjust the style and initialization of the gcode'
    # offset of axis-of-rotation relative to nozzle in z (positive if axis is in the positive z direction from the nozzle tip) when B=0
    b_offset_z: Optional[float] = None
    # values passed for initialization_data overwrite the default initialization_data of the printer
    initialization_data: Optional[dict] = {}
    save_as: Optional[str] = None
