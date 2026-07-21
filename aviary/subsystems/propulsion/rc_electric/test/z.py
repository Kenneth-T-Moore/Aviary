import unittest
from copy import deepcopy

import aviary.api as av
import numpy as np
import openmdao.api as om
from openmdao.utils.assert_utils import assert_check_partials, assert_near_equal

from aviary.subsystems.aerodynamics.UAV_Aero.custom_aero_builder import CustomAeroBuilder
from aviary.subsystems.mass.UAV_mass.mass_builder import MassBuilder as DBFMassBuilder
from aviary.models.aircraft.small_uav.phases.UAV_energy_phase import phase_info
from aviary.subsystems.propulsion.rc_electric.UAV_Builder import RCBuilder
from aviary.subsystems.propulsion.rc_electric.model.UAV_mission import RCPropMission
from aviary.subsystems.propulsion.rc_electric.model.UAV_premission import RCPropPreMission
from aviary.utils.aviary_values import AviaryValues
from aviary.variable_info.dbf_variables import Aircraft, Dynamic
from aviary.variable_info.variables import Mission
from aviary.subsystems.mass.UAV_mass.variable_info.mass_variables import Aircraft as Mass_Aircraft


#This is where you set the power balance mode for the RCPropMission. Options are 'feedforward' or 'solver'.
#Example for solver, RCBBuilder(power_balance_mode='solver')
rc_prop = RCBuilder()  # or 'solver' for the solver-based power balance mode





prob = av.AviaryProblem(verbosity=2)
prob.options['group_by_pre_opt_post'] = True
    #just selecting cruise
cruise_phase_info = {
    'pre_mission': deepcopy(phase_info['pre_mission']),
    'cruise': deepcopy(phase_info['cruise']),
    'post_mission': deepcopy(phase_info['post_mission']),
}

prob.load_inputs(
    'validation_cases/validation_data/test_models/small_scale_uav.csv',
    cruise_phase_info
)

prob.load_external_subsystems(external_subsystems=[rc_prop, CustomAeroBuilder(), DBFMassBuilder()])


prob.check_and_preprocess_inputs()



prob.build_model()



prob.add_driver('SNOPT', use_coloring=True, max_iter=250)
# prob.add_driver('IPOPT', use_coloring=True, max_iter=250)
# prob.driver.opt_settings['print_level'] = 5
# prob.driver.opt_settings['mu_strategy'] = 'adaptive'
# prob.driver.opt_settings['tol'] = 1e-6
# prob.driver.opt_settings['acceptable_tol'] = 5e-6
# prob.driver.opt_settings['constr_viol_tol'] = 1e-6
# prob.driver.opt_settings['acceptable_constr_viol_tol'] = 5e-6
# prob.driver.options['debug_print'] = []

prob.add_design_variables()

prob.add_objective(objective_type='time')


prob.setup()



prob.set_solver_print(level=0)
prob.set_initial_guesses()

# prob.set_val('traj.cruise.states:mass', 4.1, units='kg')

prob.set_val('traj.cruise.controls:rpm_slack', 4000.0, units='rpm')
prob.set_val('traj.cruise.controls:throttle', 0.3)

prob.run_aviary_problem(run_driver=True)

print(prob.get_val('traj.cruise.rhs_all.thrust_required', units='lbf'))
print(prob.get_val('traj.cruise.rhs_all.thrust_residual', units='lbf'))
print(prob.get_val('traj.cruise.rhs_all.drag', units='lbf'))
print(prob.get_val('traj.cruise.rhs_all.thrust_net_total', units='lbf'))

prob.model.list_vars(units=True, print_arrays=True)
prob.list_driver_vars(print_arrays=True)
