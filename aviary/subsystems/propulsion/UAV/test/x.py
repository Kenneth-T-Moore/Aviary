import numpy as np
import matplotlib.pyplot as plt

import openmdao.api as om

from aviary.mission.energy_state.ode.mission_EOM import MissionEOM
from aviary.subsystems.atmosphere.atmosphere import Atmosphere
from aviary.subsystems.propulsion.UAV.model.UAV_mission import UAVPropMission
from aviary.variable_info.enums import SpeedType

from aviary.variable_info.UAV_variables import Aircraft, Dynamic


class EnergyStateODE(om.Group):
    """The base class for all energy method ODE components."""

    def initialize(self):
        super().initialize()

        # TODO throttle enforcement & allocation should be moved to BaseODE for use in 2DOF
        self.options.declare('num_nodes', default=1, types=int)

    def setup(self):
        options = self.options
        nn = options['num_nodes']

        self.add_subsystem(
            name='atmosphere',
            subsys=Atmosphere(num_nodes=nn, input_speed_type=SpeedType.MACH),
            promotes=['*'],
        )

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

        sub1 = self.add_subsystem('solver_sub', om.Group(), promotes=['*'])

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
        # TODO: Need a thrust residual ref in the phase_info.
        thrust_res_ref = 1e3

        sub1.add_subsystem('rc_engine', UAVPropMission(num_nodes=1), promotes=['*'])

        # Single Engine

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
                #lower=0.0,
                #upper=1.0,
                res_ref=thrust_res_ref,
            ),
            promotes_inputs=['*'],
            promotes_outputs=['*'],
        )

        self.set_input_defaults(
            Dynamic.Vehicle.Propulsion.THROTTLE, val=np.ones(nn), units='unitless'
        )

        self.set_input_defaults(Dynamic.Atmosphere.MACH, val=np.ones(nn), units='unitless')
        self.set_input_defaults(Dynamic.Vehicle.MASS, val=np.ones(nn), units='kg')
        self.set_input_defaults(Dynamic.Mission.VELOCITY, val=np.ones(nn), units='m/s')
        self.set_input_defaults(Dynamic.Mission.ALTITUDE, val=np.ones(nn), units='m')
        self.set_input_defaults(Dynamic.Mission.ALTITUDE_RATE, val=np.ones(nn), units='m/s')

        sub1.nonlinear_solver = om.NewtonSolver(
            solve_subsystems=True,
            atol=1.0e-10,
            rtol=1.0e-10,
        )
        print_level = 2

        sub1.nonlinear_solver.linesearch = om.BoundsEnforceLS()
        sub1.linear_solver = om.DirectSolver(assemble_jac=True)
        sub1.nonlinear_solver.options['err_on_non_converge'] = True
        sub1.nonlinear_solver.options['iprint'] = print_level
        sub1.nonlinear_solver.options['maxiter'] = 150

        self.options['auto_order'] = True

        sub1.connect('thrust_net', 'thrust_net_total')

prob = om.Problem()
model = prob.model

model.add_subsystem('ode', EnergyStateODE(num_nodes=1), promotes=['*'])

model.connect('rotations_per_minute', 'rpm_slack')
model.set_input_defaults(Dynamic.Mission.VELOCITY, units='mi/h')

prob.setup()

prob.set_val(Dynamic.Mission.ALTITUDE, 200.0, units='ft')
#prob.set_val(Dynamic.Mission.VELOCITY, 0, units='mi/h')
#prob.set_val(f'battery.{Aircraft.Battery.MASS}', 2.0, units='lbm')
#prob.set_val(Aircraft.Engine.Motor.MASS, 1.0362, units='lbm')
prob.set_val(Aircraft.Engine.Motor.IDLE_CURRENT, 2.2, units='A')
prob.set_val(Aircraft.Engine.Motor.KV, 400, units='rpm/V')
prob.set_val(Aircraft.Engine.Motor.RESISTANCE, 0.05, units='ohm')
prob.set_val(Aircraft.Engine.Propeller.DIAMETER, 16, units='inch')
prob.set_val(Aircraft.Engine.Propeller.PITCH, 12, units='inch')
prob.set_val(Dynamic.Atmosphere.DENSITY, 1.225, units='kg/m**3')
prob.set_val("drag", 1.0, units='lbf')
prob.set_val("mach", 0.02)
prob.set_val("mach_rate", 0.0)
prob.set_val(Dynamic.Vehicle.Propulsion.THROTTLE, 0.5)

prob.run_model()

throttles = np.linspace(0.1, 1.0, num=10)
thrusts = np.zeros(throttles.shape)
esc_voltages = np.zeros(throttles.shape)
esc_powers = np.zeros(throttles.shape)
esc_effs = np.zeros(throttles.shape)
rpms = np.zeros(throttles.shape)
cps = np.zeros(throttles.shape)
cts = np.zeros(throttles.shape)
motor_powers = np.zeros(throttles.shape)
battery_voltages = np.zeros(throttles.shape)

prob.model.list_vars(units=True, print_arrays=True)

for j, throttle in enumerate(throttles):
    prob.set_val(Dynamic.Vehicle.Propulsion.THROTTLE, throttle)
    prob.run_model()
    thrust = prob.get_val('thrust_net', units='lbf')[0]
    thrusts[j] = thrust
    esc_voltage = prob.get_val('esc.voltage_out', units='V')[0]
    esc_voltages[j] = esc_voltage
    esc_power = prob.get_val('esc.power', units='W')[0]
    esc_powers[j] = esc_power
    esc_eff = prob.get_val('esc.efficiency')[0]
    esc_effs[j] = esc_eff
    rpm = prob.get_val(Dynamic.Vehicle.Propulsion.RPM)[0]
    rpms[j] = rpm
    ct = prob.get_val('ct')[0]
    cp = prob.get_val('cp')[0]
    cts[j] = ct
    cps[j] = cp
    motor_power = prob.get_val('motor.power', units='W')[0]
    motor_powers[j] = motor_power

    battery_volt = prob.get_val('battery.voltage_out', units='V')[0]
    battery_voltages[j] = battery_volt

plt.figure()
plt.plot(throttles, battery_voltages, '+-')
plt.xlabel('throttle')
plt.ylabel('Battery Voltage (V)')
plt.grid()

plt.figure()
plt.plot(throttles, thrusts, '+-')
plt.xlabel('throttle')
plt.ylabel('thrust (lbf)')
plt.grid()

plt.figure()
plt.plot(throttles, esc_voltages, '+-')
plt.xlabel('throttle')
plt.ylabel('ESC Voltage (V)')
plt.grid()

plt.figure()
plt.plot(throttles, cts, '+-', label='CT')
plt.plot(throttles, cps, '+-', label='CP')
plt.xlabel('throttle')
plt.ylabel('CT and CP')
plt.legend()
plt.grid()

plt.figure()
plt.plot(throttles, rpms, '+-')
plt.xlabel('throttle')
plt.ylabel('RPM')
plt.grid()

plt.figure()
plt.plot(throttles, esc_powers, '+-')
plt.xlabel('throttle')
plt.ylabel('ESC Power (W)')
plt.grid()

plt.figure()
plt.plot(throttles, esc_effs, '+-')
plt.xlabel('throttle')
plt.ylabel('ESC Efficiency')
plt.grid()

plt.figure()
plt.plot(throttles, motor_powers, '+-')
plt.xlabel('throttle')
plt.ylabel('Motor Power (W)')
plt.grid()

plt.show()

# plt

print('done')



