import unittest
from copy import deepcopy

import openmdao.api as om

import aviary.api as av

from aviary.models.missions.energy_state_default import phase_info

from aviary.api import AviaryProblem
from aviary.subsystems.test.test_dummy_subsystem import FullSubsystemBuilder, ExtendedMetaData

prob = AviaryProblem(verbosity=0, meta_data=ExtendedMetaData)

prob.load_inputs(
    'models/aircraft/advanced_single_aisle/advanced_single_aisle_FLOPS.csv', phase_info
)

prob.load_external_subsystems([FullSubsystemBuilder()])
prob.check_and_preprocess_inputs()
prob.build_model()
prob.add_driver()
prob.add_design_variables()
prob.add_objective()
prob.setup()
prob.run_aviary_problem()
# om.n2(prob, show_browser=False)
prob.model.list_vars(units=True, print_arrays=True)
prob.get_val('traj.phases.climb.rhs_all.solver_sub.full_suite.aircraft:dummy_mission_input')
