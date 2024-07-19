from typing import Optional
from fullcontrol.common import Point as BasePoint
from copy import deepcopy

class Point(BasePoint):
    'generic gcode Point with 4-axis aspects added/modified'
    b: Optional[float] = None
    c: Optional[float] = None

    def CXZB_gcode(self, self_system_point, p) -> float:
        'generate CXZB gcode string to move from a point p to this point. return XYZB string'
        s = ''
        if self_system_point.x != None and self_system_point.x != p.x:
            s += f'X{round(self_system_point.x, 6):.6} '
        if self_system_point.z != None and self_system_point.z != p.z:
            s += f'Z{round(self_system_point.z, 6):.6} '
        if self_system_point.b != None and self_system_point.b != p.b:
            s += f'B{round(self_system_point.b, 6):.6} '
        if self_system_point.c != None and self_system_point.c != p.c:
            s += f'C{round(self_system_point.c, 6):.6} '
        return s if s != '' else None

    def inverse_kinematics(self, state):
        'calculate system point for the current point (in part coordinates)'

        def model2system(model_point, state, system_type: str):
            from math import cos, sin, radians, degrees, sqrt, atan2
            system_point = deepcopy(model_point)
            if system_type == 'CXZB':
                # calculate CXY as if B=0 first:
                x_for_b0 = sqrt(model_point.x ** 2 + model_point.y ** 2)
                c_for_b0 = 0.0 if x_for_b0 == 0.0 else -degrees(atan2(model_point.y, model_point.x))
                z_for_b0 = model_point.z
                
                # now calculate offset of nozzle caused by B rotation:
                x_offset_from_b = sin(radians(model_point.b)) * state.printer.b_offset_z
                z_offset_from_b = state.printer.b_offset_z - cos(radians(model_point.b)) * state.printer.b_offset_z
                
                # now apply offset to CXY
                c_system = c_for_b0
                x_system = x_for_b0 - x_offset_from_b
                z_system = z_for_b0 - z_offset_from_b

            system_point.c = round(c_system, 6)
            system_point.x = round(x_system, 6)
            system_point.z = round(z_system, 6)
            system_point.b = round(model_point.b, 6)
            return system_point

        # make sure undefined attributes of the current point (self) are taken from the point in state
        model_point = deepcopy(state.point)
        model_point.update_from(self)
        # inverse kinematics:
        system_point = model2system(model_point, state, 'CXZB')
        return system_point

    def gcode(self, state):
        'process this instance in a list of steps supplied by the designer to generate and return a line of gcode'
        self_system_point = self.inverse_kinematics(state)
        CXZB_str = self.CXZB_gcode(self_system_point, state.system_point)
        if CXZB_str != None:  # only write a line of gcode if movement occurs
            G_str = 'G1 ' if state.extruder.on else 'G0 '
            F_str = state.printer.f_gcode(state)
            E_str = state.extruder.e_gcode(self, state)
            gcode_str = f'{G_str}{F_str}{CXZB_str}{E_str}'
            state.printer.speed_changed = False
            state.point.update_from(self)
            state.system_point.update_from(self_system_point)
            return gcode_str.strip()  # strip the final space
