from typing import Optional
from fullcontrol.common import Printer as BasePrinter
from fullcontrol.common import Point


class Printer(BasePrinter):
    'generic gcode Printer with 4-axis aspects added/modified'
    command_list: Optional[dict] = None
    new_command: Optional[dict] = None
    speed_changed: Optional[bool] = None
    # see controls.py for definition of the following terms
    b_offset_z: Optional[float] = None

    def f_gcode(self, state):
        if self.speed_changed == True:
            return f'F{self.print_speed} ' if state.extruder.on else f'F{self.travel_speed} '
        else:
            return ''

    def gcode(self, state):
        'process this instance in a list of steps supplied by the designer to generate and return a line of gcode'
        # update all attributes of the tracking instance with the new instance (self)
        state.printer.update_from(self)
        if self.print_speed != None \
                or self.travel_speed != None:
            state.printer.speed_changed = True
        if self.new_command != None:
            state.printer.command_list = {
                **state.printer.command_list, **self.new_command}
