import numpy as np
import openmdao.api as om

from aviary.mission.base_ode import BaseODE as _BaseODE
from aviary.mission.flops_based.ode.mission_EOM import MissionEOM

from aviary.subsystems.propulsion.throttle_allocation import ThrottleAllocator
from aviary.variable_info.enums import SpeedType, ThrottleAllocation
from aviary.variable_info.variables import Aircraft, Dynamic, Mission


class EnergyODE(_BaseODE):
    """The base class for all energy method ODE components."""

    def initialize(self):
        super().initialize()

        self.options.declare(
            'use_actual_takeoff_mass',
            default=False,
            desc='flag to use actual takeoff mass in the climb phase, otherwise assume 100 kg fuel burn',
        )
        # TODO throttle enforcement & allocation should be moved to BaseODE for
        # use in 2DOF
        self.options.declare(
            'throttle_enforcement',
            default='path_constraint',
            values=['path_constraint', 'boundary_constraint', 'bounded', None],
            desc='flag to enforce throttle constraints on the path or at the segment boundaries or using solver bounds',
        )
        self.options.declare(
            'throttle_allocation',
            default=ThrottleAllocation.FIXED,
            types=ThrottleAllocation,
            desc='Flag that determines how to handle throttles for multiple engines.',
        )
        self.options.declare(
            'throttle_optimize',
            default=False,
            types=bool,
            desc='Set to True to let optimizer handle throttle.',
        )

    def setup(self):
        options = self.options
        nn = options['num_nodes']
        aviary_options = options['aviary_options']
        num_engine_type = len(aviary_options.get_val(Aircraft.Engine.NUM_ENGINES))

        self.add_atmosphere(input_speed_type=SpeedType.MACH)

        # add execcomp to compute velocity_rate based off mach_rate and sos
        self.add_subsystem(
            name='velocity_rate_comp',
            subsys=om.ExecComp(
                'velocity_rate = mach_rate * sos',
                mach_rate={'units': '1/s', 'shape': (nn,)},
                sos={'units': 'm/s', 'shape': (nn,)},
                velocity_rate={'units': 'm/s**2', 'shape': (nn,)},
                has_diag_partials=True,
            ),
            promotes_inputs=[
                ('mach_rate', Dynamic.Atmosphere.MACH_RATE),
                ('sos', Dynamic.Atmosphere.SPEED_OF_SOUND),
            ],
            promotes_outputs=[('velocity_rate', Dynamic.Mission.VELOCITY_RATE)],
        )

        throttle_optimize = options['throttle_optimize']

        sub1 = self.add_subsystem('solver_sub', om.Group(), promotes=['*'])

        if throttle_optimize:
            solver_group = None
        else:
            solver_group = sub1

        self.add_core_subsystems(solver_group=solver_group)

        self.add_external_subsystems(solver_group=sub1)

        sub1.add_subsystem(
            name='mission_EOM',
            subsys=MissionEOM(num_nodes=nn),
            promotes_inputs=[
                Dynamic.Mission.VELOCITY,
                Dynamic.Vehicle.MASS,
                Dynamic.Vehicle.Propulsion.THRUST_MAX_TOTAL,
                Dynamic.Vehicle.DRAG,
                Dynamic.Mission.ALTITUDE_RATE,
                Dynamic.Mission.VELOCITY_RATE,
            ],
            promotes_outputs=[
                Dynamic.Mission.SPECIFIC_ENERGY_RATE_EXCESS,
                Dynamic.Mission.ALTITUDE_RATE_MAX,
                Dynamic.Mission.DISTANCE_RATE,
                'thrust_required',
            ],
        )

        # THROTTLE Section
        # TODO: Split this out into a function that can be used by the other ODEs.
        if num_engine_type > 1:
            # Multi Engine

            sub1.add_subsystem(
                name='throttle_balance',
                subsys=om.BalanceComp(
                    name='aggregate_throttle',
                    units='unitless',
                    val=np.ones((nn,)),
                    lhs_name='thrust_required',
                    rhs_name=Dynamic.Vehicle.Propulsion.THRUST_TOTAL,
                    eq_units='lbf',
                    normalize=False,
                    res_ref=1.0e6,
                ),
                promotes_inputs=['*'],
                promotes_outputs=['*'],
            )

            sub1.add_subsystem(
                'throttle_allocator',
                ThrottleAllocator(
                    num_nodes=nn, throttle_allocation=self.options['throttle_allocation']
                ),
                promotes_inputs=['*'],
                promotes_outputs=['*'],
            )

        else:
            # Single Engine

            if throttle_optimize:
                self.add_subsystem(
                    'throttle_balance',
                    om.ExecComp(
                        'thrust_residual=thrust_required-thrust',
                        thrust={'val': np.ones((nn, ))},
                        thrust_required={'val': np.ones((nn, ))},
                        thrust_residual={'val': np.ones((nn, ))},
                    ),
                    promotes_inputs=[('thrust', Dynamic.Vehicle.Propulsion.THRUST_TOTAL), 'thrust_required'],
                    promotes_outputs=['*']
                )
                self.add_constraint('thrust_residual', ref=10000.0)
            else:

                # Add a balance comp to compute throttle based on the required thrust.
                sub1.add_subsystem(
                    name='throttle_balance',
                    subsys=om.BalanceComp(
                        name=Dynamic.Vehicle.Propulsion.THROTTLE,
                        units='unitless',
                        val=np.ones((nn,)),
                        lhs_name='thrust_required',
                        rhs_name=Dynamic.Vehicle.Propulsion.THRUST_TOTAL,
                        eq_units='lbf',
                        normalize=False,
                        lower=0.0 if options['throttle_enforcement'] == 'bounded' else None,
                        upper=1.0 if options['throttle_enforcement'] == 'bounded' else None,
                        res_ref=1.0e6,
                    ),
                    promotes_inputs=['*'],
                    promotes_outputs=['*'],
                )

            self.set_input_defaults(Dynamic.Vehicle.Propulsion.THROTTLE, val=1.0, units='unitless')

        self.set_input_defaults(Dynamic.Atmosphere.MACH, val=np.ones(nn), units='unitless')
        self.set_input_defaults(Dynamic.Vehicle.MASS, val=np.ones(nn), units='kg')
        self.set_input_defaults(Dynamic.Mission.VELOCITY, val=np.ones(nn), units='m/s')
        self.set_input_defaults(Dynamic.Mission.ALTITUDE, val=np.ones(nn), units='m')
        self.set_input_defaults(Dynamic.Mission.ALTITUDE_RATE, val=np.ones(nn), units='m/s')

        if options['use_actual_takeoff_mass']:
            exec_comp_string = 'initial_mass_residual = initial_mass - mass[0]'
            initial_mass_string = Mission.Takeoff.FINAL_MASS
        else:
            exec_comp_string = 'initial_mass_residual = initial_mass - mass[0] - 100.'
            initial_mass_string = Mission.Summary.GROSS_MASS

        # Experimental: Add a component to constrain the initial mass to be equal
        # to design gross weight.
        initial_mass_residual_constraint = om.ExecComp(
            exec_comp_string,
            initial_mass={'units': 'kg'},
            mass={'units': 'kg', 'shape': (nn,)},
            initial_mass_residual={'units': 'kg', 'res_ref': 1.0e5},
        )

        self.add_subsystem(
            'initial_mass_residual_constraint',
            initial_mass_residual_constraint,
            promotes_inputs=[
                ('initial_mass', initial_mass_string),
                ('mass', Dynamic.Vehicle.MASS),
            ],
            promotes_outputs=['initial_mass_residual'],
        )

        print_level = 2

        if not throttle_optimize:
            sub1.nonlinear_solver = om.NewtonSolver(
                solve_subsystems=True,
                atol=1.0e-10,
                rtol=1.0e-10,
            )
            sub1.nonlinear_solver.linesearch = None
            sub1.linear_solver = om.DirectSolver(assemble_jac=True)
            sub1.nonlinear_solver.options['err_on_non_converge'] = False
            sub1.nonlinear_solver.options['iprint'] = print_level
            sub1.nonlinear_solver.options['debug_print'] = True

        self.options['auto_order'] = True
