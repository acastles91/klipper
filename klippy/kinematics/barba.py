# Code for handling the kinematics of cartesian robots
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import stepper
from . import idex_modes

class Barba:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        self.toolhead1_motors = [stepper.LookupMultiRail(config.getsection('stepper_' + n)) 
                                for n in 'ab']
        for rail, axis in zip(self.toolhead1_motors, 'ab'):
            rail.setup_itersolve('cartesian_stepper_alloc', axis.encode())
        
        ranges = [r.get_range() for r in self.toolhead1_motors]
        self.axes_min = toolhead.AB_Coord(*[r[0] for r in ranges])
        self.axes_max = toolhead.AB_Coord(*[r[1] for r in ranges])

        for s in self.get_steppers():
                    s.set_trapq(toolhead.get_trapq())
                    toolhead.register_step_generator(s.generate_steps)
        self.printer.register_event_handler("stepper_enable:motor_off",
                                            self._motor_off)
        
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.limits = [(1.0, -1.0)] * 2
        
    def get_steppers(self):
        return [s for rail in self.toolhead1_motors for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):
            return [stepper_positions[rail.get_name()] for rail in self.toolhead1_motors]
    def update_limits(self, i, range):
        l, h = self.limits[i]
        # Only update limits if this axis was already homed,
        # otherwise leave in un-homed state.
        if l <= h:
            self.limits[i] = range
    def override_rail(self, i, rail):
        self.rails[i] = rail
    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.toolhead1_motors):
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()
    def home_axis(self, homing_state, axis, rail):
        # Determine movement
        logging.info("homing_axis from kinematics")
        position_min, position_max = rail.get_range()
        # hi = homing info
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        # Perform homing
        homing_state.home_rails_barba([rail], forcepos, homepos)
    def home(self, homing_state):
        # Each axis is homed independently and in order
        logging.info("home en barba")
        for axis in homing_state.get_axes():
            logging.info("axes " + str(axis))
            self.home_axis(homing_state, axis, self.toolhead1_motors[axis])
    def _motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * 2
    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1):
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("ab", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return Barba(toolhead, config)