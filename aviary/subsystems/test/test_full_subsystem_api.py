import unittest
from copy import deepcopy

import openmdao.api as om

import aviary.api as av

from aviary.models.missions.energy_state_default import phase_info

from aviary.api import AviaryProblem
from aviary.subsystems.test.test_dummy_subsystem import FullSubsystemBuilder

prob = AviaryProblem(verbosity=0)

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
