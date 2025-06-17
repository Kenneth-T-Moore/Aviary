"""
Run the detailed landing aviary model as an external subsystem inside of an Aviary sizing
model.
"""
import numpy as np

import openmdao.api as om

from aviary.api import Mission
from aviary.examples.external_subsystems.detailed_landing.detailed_landing_builder import (
    DetailedLandingBuilder,
)
from aviary.interface.methods_for_level2 import AviaryProblem
from aviary.variable_info.enums import Verbosity
from aviary.variable_info.variables import Aircraft, Dynamic, Mission, Settings


#
phase_info = {
    'pre_mission': {'include_takeoff': True, 'optimize_mass': True},
    'climb': {
        'subsystem_options': {'core_aerodynamics': {'method': 'computed'}},
        'user_options': {
            'fix_initial': False,
            'input_initial': True,
            'optimize_mach': True,
            'optimize_altitude': True,
            'use_polynomial_control': False,
            'num_segments': 6,
            'order': 3,
            'solve_for_distance': False,
            'initial_mach': (0.3, 'unitless'),
            'final_mach': (0.79, 'unitless'),
            'mach_bounds': ((0.1, 0.8), 'unitless'),
            'initial_altitude': (35.0, 'ft'),
            'final_altitude': (35000.0, 'ft'),
            'altitude_bounds': ((0.0, 35000.0), 'ft'),
            'throttle_enforcement': 'path_constraint',
            'constrain_final': False,
            'fix_duration': False,
            'initial_bounds': ((0.0, 2.0), 'min'),
            'duration_bounds': ((5.0, 50.0), 'min'),
            'no_descent': False,
            'add_initial_mass_constraint': False,
        },
        'initial_guesses': {'time': ([0, 40.0], 'min')},
    },
    'cruise': {
        'subsystem_options': {'core_aerodynamics': {'method': 'computed'}},
        'user_options': {
            'optimize_mach': True,
            'optimize_altitude': True,
            'polynomial_control_order': 1,
            'use_polynomial_control': True,
            'num_segments': 1,
            'order': 3,
            'solve_for_distance': False,
            'initial_mach': (0.79, 'unitless'),
            'final_mach': (0.79, 'unitless'),
            'mach_bounds': ((0.79, 0.79), 'unitless'),
            'initial_altitude': (35000.0, 'ft'),
            'final_altitude': (35000.0, 'ft'),
            'altitude_bounds': ((35000.0, 35000.0), 'ft'),
            'throttle_enforcement': 'boundary_constraint',
            'fix_initial': False,
            'constrain_final': False,
            'fix_duration': False,
            'initial_bounds': ((64.0, 192.0), 'min'),
            'duration_bounds': ((60.0, 720.0), 'min'),
        },
        'initial_guesses': {'time': ([128, 113], 'min')},
    },
    'descent': {
        'subsystem_options': {'core_aerodynamics': {'method': 'computed'}},
        'user_options': {
            'optimize_mach': True,
            'optimize_altitude': True,
            'use_polynomial_control': False,
            'num_segments': 5,
            'order': 3,
            'solve_for_distance': False,
            'initial_mach': (0.79, 'unitless'),
            'final_mach': (0.3, 'unitless'),
            'mach_bounds': ((0.2, 0.8), 'unitless'),
            'initial_altitude': (35000.0, 'ft'),
            'final_altitude': (35.0, 'ft'),
            'altitude_bounds': ((0.0, 35000.0), 'ft'),
            'throttle_enforcement': 'path_constraint',
            'fix_initial': False,
            'constrain_final': True,
            'fix_duration': False,
            'initial_bounds': ((120.0, 800.0), 'min'),
            'duration_bounds': ((5.0, 35.0), 'min'),
            'no_climb': True,
        },
        'initial_guesses': {'time': ([241, 30], 'min')},
    },
    'post_mission': {
        'include_landing': True,
        'constrain_range': True,
        'target_range': (3375.0, 'nmi'),
    },
}


aircraft_filename = "models/test_aircraft/aircraft_for_bench_FwFm.csv"
prob = AviaryProblem()

phase_info['post_mission']['external_subsystems'] = [DetailedLandingBuilder()]

prob.load_inputs(aircraft_filename, phase_info, verbosity=1)

prob.aviary_inputs.set_val(Settings.VERBOSITY, 1)
prob.check_and_preprocess_inputs()
prob.add_pre_mission_systems()
prob.add_phases()
prob.add_post_mission_systems()
prob.link_phases()

prob.add_driver("SNOPT", max_iter=50)
prob.driver.opt_settings["Major optimality tolerance"] = 1e-4
prob.driver.opt_settings["Major feasibility tolerance"] = 1e-4

prob.add_design_variables()

prob.add_objective(objective_type='fuel_burned')

prob.options['group_by_pre_opt_post'] = True

prob.model.connect(
    'traj.descent.states:mass',
    'detailed_landing.mass_start_landing',
    src_indices=[-1]
)

prob.setup()

prob.set_initial_guesses()

prob.final_setup()

#prob.run_model()
#prob.model.list_vars(units=True, print_arrays=True)
#prob.list_driver_vars(print_arrays=True, driver_scaling=False, desvar_opts=['lower', 'upper', 'ref', 'ref0', 'units'], cons_opts=['lower', 'upper', 'ref', 'ref0', 'units'])
#exit()

prob.failed = prob.run_aviary_problem(simulate=False, optimization_history_filename='z.sql')

#prob.model.list_vars(units=True, print_arrays=True)
#prob.list_driver_vars(print_arrays=True, driver_scaling=False, desvar_opts=['lower', 'upper', 'ref', 'ref0', 'units'], cons_opts=['lower', 'upper', 'ref', 'ref0', 'units'])

print('done')
