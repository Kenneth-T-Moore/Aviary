"""Run the a mission with a simple external component that computes the wing and horizontal tail mass."""

from copy import deepcopy

import matplotlib.pyplot as plt
import aviary.api as av
from aviary.examples.external_subsystems.dbf_based_mass.dbf_mass_builder import DBFMassBuilder
from aviary.examples.external_subsystems.dbf_based_geometry.dbf_geometry_builder import DBFGeometryBuilder
from aviary.variable_info.variable_meta_data import _MetaData

phase_info = deepcopy(av.default_height_energy_phase_info)
# Here we just add the dbf mass and geom subsystems to only the pre-mission
phase_info['pre_mission']['external_subsystems'] = [ DBFGeometryBuilder(), DBFMassBuilder()]

if __name__ == '__main__':

    # rc = 0.508

    # x = []
    # y = []

    # for i in range(10):
    #     rc += 1e-5
    #     x.append(rc)
    #     print(rc)

    prob = av.AviaryProblem()

    # Load aircraft and options data from user
    # Allow for user overrides here
    prob.load_inputs('models/aircraft/test_aircraft/dbf_geom_and_mass_aircraft.csv', phase_info, meta_data=_MetaData)

    # Preprocess inputs
    prob.check_and_preprocess_inputs()

    prob.add_pre_mission_systems()

    prob.add_phases()

    prob.add_post_mission_systems()

    # Link phases and variables
    prob.link_phases()

    prob.add_driver('IPOPT', max_iter=200)

    # prob.add_design_variables()
    prob.model.add_design_var(av.Aircraft.Wing.ROOT_CHORD, lower=1e-5, upper=1e2)

    # prob.add_objective()
    prob.model.add_objective(av.Aircraft.Wing.MASS, scaler=-1)

    prob.setup()

    # prob.model.set_val(av.Aircraft.Wing.ROOT_CHORD, val=rc, units='m')

    prob.set_initial_guesses()

    # prob.run_model()
    prob.run_aviary_problem(suppress_solver_print=True, verbosity=1)

    # print(f"Battery Mass: {prob.get_val(av.Aircraft.Battery.MASS):.4f} kg")
    # print(f"Motor Mass: {prob.get_val(av.Aircraft.Engine.Motor.MASS):.4f} kg")
    print('Wing Mass', prob.get_val(av.Aircraft.Wing.MASS))
    # y.append(prob.get_val(av.Aircraft.Wing.WETTED_AREA)[0])
    # print('Horizontal Tail Mass', prob.get_val(av.Aircraft.HorizontalTail.MASS))
    # print('Vertical Tail Mass', prob.get_val(av.Aircraft.VerticalTail.MASS))
    # print('Fuselage Mass', prob.get_val(av.Aircraft.Fuselage.MASS))
    # print('Structure Mass', prob.get_val(av.Aircraft.Design.STRUCTURE_MASS))
    # print('Zero Fuel Mass', prob.get_val(av.Aircraft.Design.ZERO_FUEL_MASS))
    # print('Operating Mass', prob.get_val(av.Aircraft.Design.OPERATING_MASS))
    # print('Gross Mass', prob.get_val(av.Mission.Summary.GROSS_MASS))

    # with open("variables.txt", "w") as f:
    #     prob.model.list_vars(out_stream=f)
    # prob.list_driver_vars()

    # plt.plot(x,y)
    # plt.xlabel("Root Chord [m]")
    # plt.ylabel("Wing Mass [kg]")
    # plt.title("Wing Mass vs Root Chord")
    # plt.grid(True)

    # plt.show()
    print('done')