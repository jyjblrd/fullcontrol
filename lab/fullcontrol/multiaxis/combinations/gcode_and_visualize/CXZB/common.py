
from typing import Union

# # see comment in __init__.py about why this module exists

# # import functions and classes that will be accessible to the user
from fullcontrol.common import check
# don't import all geometry functions since they are not designed for multiaxis Points
from fullcontrol.geometry import move, move_polar, travel_to
from fullcontrol.combinations.gcode_and_visualize.classes import *
from .classes import *  # over-write above imports
import fullcontrol.geometry as xyz_geom
from .xyz_add_b import xyz_add_b


def transform(steps: list, result_type: str, controls: Union[GcodeControls, PlotControls] = None):
    '''transform a fullcontrol design (a list of function class instances) into result_type
    "gcode" or "plot". Optionally, GcodeControls or PlotControls can be passed to control 
    how the gcode or plot are generated.
    '''

    if result_type == 'gcode':
        from lab.fullcontrol.multiaxis.gcode.CXZB.steps2gcode import gcode
        if controls != None:
            return gcode(steps, controls)
        else:
            return gcode(steps)

    elif result_type == 'plot':
        from fullcontrol.visualize.steps2visualization import visualize
        if controls != None:
            return visualize(steps, controls)
        else:
            return visualize(steps)
