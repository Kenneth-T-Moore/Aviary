"""
Group containing a submodel component with detailed landing.
"""
from copy import copy

import openmdao.api as om

import aviary.api as av
from aviary.variable_info.enums import EquationsOfMotion
from aviary.variable_info.variables import Aircraft, Mission, Settings


# TODO: Read from external (experimental) file.
landing_aero = {
    'core_aerodynamics': {
        'method': 'low_speed',
        'ground_altitude': 0.0,  # units='ft'
        'angles_of_attack': [
            -5.0, -4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0,
            6.0,  7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
        ],  # units='deg'
        'lift_coefficients': [
            0.01, 0.1, 0.2, 0.3, 0.4, 0.5178, 0.6, 0.75, 0.85, 0.95, 1.05,
            1.15, 1.25, 1.35, 1.5, 1.6, 1.7, 1.8, 1.85, 1.9, 1.95,
        ],
        'drag_coefficients': [
            0.04, 0.02, 0.01, 0.02, 0.04, 0.0674, 0.065, 0.065, 0.07, 0.072,
            0.076, 0.084, 0.09, 0.10, 0.11, 0.12, 0.13, 0.15, 0.16, 0.18, 0.20,
        ],
        'lift_coefficient_factor': 2.0,
        'drag_coefficient_factor': 3.0,
    }
}


phase_info = {
    'pre_mission': {'include_takeoff': False, 'optimize_mass': False},
    'GH': {
        'user_options': {
            'num_segments': 5,
            'order': 3,
            'fix_initial': True,
            'ground_roll': False,
            'clean': False,
            'initial_ref': (1.0e3, 'ft'),
            'initial_bounds': ((0.0, 16.0e3), 'ft'),
            'duration_ref': (1.0e3, 'ft'),
            'duration_bounds': ((500.0, 5.0e3), 'ft'),
            'mach_bounds': ((0.1, 0.5), 'unitless'),
            'initial_mach': (0.15, 'unitless'),
            'final_mach': (0.15, 'unitless'),
            'initial_altitude': (500.0, 'ft'),
            'final_altitude': (394.0, 'ft'),
            'altitude_bounds': ((0.0, 1000.0), 'ft'),
            'polynomial_control_order': 1,
            'throttle_enforcement': 'bounded',
            'optimize_mach': False,
            'optimize_altitude': False,
            'rotation': False,
            'constraints': {
                'flight_path_angle': {
                    'equals': -3.0,
                    'loc': 'initial',
                    'units': 'deg',
                    'type': 'boundary',
                },
            },
        },
        'subsystem_options': landing_aero,
        'initial_guesses': {
            'distance': [(0.0e3, 2.0e3), 'ft'],
            'time': [(0.0, 12.0), 's'],
            'mass': [(120.0e3, 119.8e3), 'lbm'],
        },
    },
    'HI': {
        'user_options': {
            'num_segments': 5,
            'order': 3,
            'fix_initial': False,
            'ground_roll': False,
            'clean': False,
            'initial_ref': (1.0e3, 'ft'),
            'initial_bounds': ((0.0, 16.0e3), 'ft'),
            'duration_ref': (1.0e3, 'ft'),
            'duration_bounds': ((500.0, 15.0e3), 'ft'),
            'mach_bounds': ((0.1, 0.5), 'unitless'),
            'altitude_bounds': ((0.0, 1000.0), 'ft'),
            'initial_mach': (0.15, 'unitless'),
            'final_mach': (0.15, 'unitless'),
            'initial_altitude': (394.0, 'ft'),
            'final_altitude': (50.0, 'ft'),
            'polynomial_control_order': 1,
            'throttle_enforcement': 'bounded',
            'optimize_mach': False,
            'optimize_altitude': False,
            'rotation': False,
            'constraints': {
                'flight_path_angle': {
                    'equals': -3.0,
                    'loc': 'final',
                    'units': 'deg',
                    'type': 'boundary',
                },
            },
        },
        'subsystem_options': landing_aero,
        'initial_guesses': {
            'distance': [(2.0e3, 6.5e3), 'ft'],
            'time': [(12.0, 50.0), 's'],
            'mass': [(119.8e3, 119.7e3), 'lbm'],
        },
    },
    'IJ': {
        'user_options': {
            'num_segments': 5,
            'order': 3,
            'fix_initial': False,
            'ground_roll': False,
            'clean': False,
            'initial_ref': (1.0e3, 'ft'),
            'initial_bounds': ((0.0, 30.0e3), 'ft'),
            'duration_ref': (1.0e3, 'ft'),
            'duration_bounds': ((500.0, 15.0e3), 'ft'),
            'mach_bounds': ((0.1, 0.5), 'unitless'),
            'altitude_bounds': ((0.0, 1000.0), 'ft'),
            'initial_mach': (0.15, 'unitless'),
            'final_mach': (0.15, 'unitless'),
            'initial_altitude': (50.0, 'ft'),
            'final_altitude': (0.0, 'ft'),
            'polynomial_control_order': 2,
            'throttle_enforcement': 'path_constraint',
            'optimize_mach': False,
            'optimize_altitude': True,
            'rotation': False,
            'constraints': {},
        },
        'subsystem_options': landing_aero,
        'initial_guesses': {
            'distance': [(8.5e3, 2.0e3), 'ft'],
            'time': [(50.0, 60.0), 's'],
            'mass': [(119.7e3, 119.67e3), 'lbm'],
        },
    },
    'post_mission': {
        'include_landing': False,
        'constrain_range': False,
    },
}


def create_prob(aviary_inputs):
    """
    Return a problem
    """
    prob = av.AviaryProblem()

    # Only need shallowcopy to decouple the base.
    sub_aviary_inputs = copy(aviary_inputs)

    # Need these equations for detailed landing.
    sub_aviary_inputs.set_val(Settings.EQUATIONS_OF_MOTION, EquationsOfMotion.SOLVED_2DOF)

    # Load aircraft and options data from user
    prob.load_inputs(sub_aviary_inputs, phase_info)

    prob.check_and_preprocess_inputs()
    prob.add_pre_mission_systems()
    prob.add_phases()
    prob.add_post_mission_systems()
    prob.link_phases()

    prob.add_driver('SLSQP', max_iter=30)

    prob.add_design_variables()

    # With mass fixed, what does this do?
    prob.add_objective('mass')

    return prob


class AviarySubmodelComp(om.SubmodelComp):
    """
    We need to subclass so that we can set the initial conditions.
    """
    def setup(self):
        super().setup()

        sub = self._subprob
        sub.setup()
        sub.set_initial_guesses()
        sub.final_setup()


class ExternalLandingGroup(om.Group):

    def initialize(self):
        self.options.declare(
            'aviary_inputs',
            default=None,
            allow_none=False,
            desc="Aircraft definition."
        )

    def setup(self):
        aviary_inputs = self.options['aviary_inputs']
        subprob = create_prob(aviary_inputs)

        comp = AviarySubmodelComp(
            problem=subprob,
            inputs=[
                Mission.Summary.GROSS_MASS,
            ],
            outputs=[
                ('traj.IJ.timeseries.distance', 'distance'),
            ]
        )

        self.add_subsystem('external_landing', comp, promotes=['*'])