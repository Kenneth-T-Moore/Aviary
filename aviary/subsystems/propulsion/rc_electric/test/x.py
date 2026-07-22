import numpy as np
import matplotlib.pyplot as plt

import openmdao.api as om

from aviary.subsystems.atmosphere.atmosphere import Atmosphere
from aviary.subsystems.propulsion.rc_electric.model.UAV_mission import RCPropMission
from aviary.variable_info.dbf_variables import Aircraft, Dynamic


prob = om.Problem()
model = prob.model

model.add_subsystem('atm', Atmosphere(num_nodes=1), promotes=['*'])
model.add_subsystem('rc_engine', RCPropMission(num_nodes=1), promotes=['*'])

model.connect('rotations_per_minute', 'rpm_slack')
model.set_input_defaults(Dynamic.Mission.VELOCITY, units='kn')

prob.setup()

prob.set_val(Dynamic.Mission.ALTITUDE, 200.0, units='ft')
prob.set_val(Dynamic.Mission.VELOCITY, 0.0, units='kn')
prob.set_val(f'battery.{Aircraft.Battery.MASS}', 2.0, units='lbm')
prob.set_val(Aircraft.Engine.Motor.MASS, 1.0362, units='lbm')
prob.set_val(Aircraft.Engine.Motor.IDLE_CURRENT, 2.2, units='A')
prob.set_val(Aircraft.Engine.Motor.KV, 400, units='rpm/V')
prob.set_val(Aircraft.Engine.Motor.RESISTANCE, 0.05, units='ohm')
prob.set_val(Aircraft.Engine.Propeller.DIAMETER, 19, units='inch')
prob.set_val(Aircraft.Engine.Propeller.PITCH, 12, units='inch')
prob.set_val(Aircraft.Engine.Propeller.DIAMETER, 19, units='inch')

prob.run_model()

throttles = np.linspace(0.1, 1.0, num=10)
thrusts = np.zeros(throttles.shape)
esc_voltages = np.zeros(throttles.shape)
esc_powers = np.zeros(throttles.shape)
esc_effs = np.zeros(throttles.shape)

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

plt.plot(throttles, thrusts, '+-')
plt.xlabel('throttle')
plt.ylabel('thrust (lbf)')
plt.grid()

plt.figure()
plt.plot(throttles, esc_voltages, '+-')
plt.xlabel('throttle')
plt.ylabel('Battery Voltage (V)')
plt.grid()

plt.figure()
plt.plot(throttles, esc_powers, '+-')
plt.xlabel('throttle')
plt.ylabel('Battery Power (W)')
plt.grid()

plt.figure()
plt.plot(throttles, esc_effs, '+-')
plt.xlabel('throttle')
plt.ylabel('Battery Efficiency')
plt.grid()

plt.show()

print('done')

