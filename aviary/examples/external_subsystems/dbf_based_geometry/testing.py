import openmdao.api as om
import numpy as np

from aviary.examples.external_subsystems.dbf_based_geometry.dbf_geometry import DBFGeom
from aviary.examples.external_subsystems.dbf_based_mass.mass_summation import MassSummation
from aviary.examples.external_subsystems.dbf_based_mass.dbf_wing import DBFWingMass
from aviary.examples.external_subsystems.dbf_based_mass.dbf_horizontaltail import DBFHorizontalTailMass 
from aviary.examples.external_subsystems.dbf_based_mass.dbf_verticaltail import DBFVerticalTailMass
from aviary.examples.external_subsystems.dbf_based_mass.dbf_fuselage import DBFFuselageMass

from aviary.variable_info.variables import Aircraft

if __name__ == '__main__':
    prob = om.Problem()
    model = prob.model

    # -----------------------------
    # Add Independent Variable Component for design variables
    # -----------------------------
    indeps = model.add_subsystem('indeps', om.IndepVarComp(), promotes=['*'])
    
    # Add design variables as independent variables
    indeps.add_output(Aircraft.Wing.ROOT_CHORD, val=20.0, units='inch')
    indeps.add_output(Aircraft.Wing.SPAN, val=4.667, units='ft')
    indeps.add_output(Aircraft.Wing.VERTICAL_MOUNT_LOCATION, val=0.75, units='unitless')
    indeps.add_output(Aircraft.Wing.CENTER_DISTANCE, val=0.15, units='unitless')
    indeps.add_output(Aircraft.HorizontalTail.ROOT_CHORD, val=8.75, units='inch')
    indeps.add_output(Aircraft.HorizontalTail.SPAN, val=28.0, units='inch')
    indeps.add_output(Aircraft.VerticalTail.ROOT_CHORD, val=8.75, units='inch')
    indeps.add_output(Aircraft.VerticalTail.SPAN, val=1.0, units='ft')
    indeps.add_output(Aircraft.Fuselage.LENGTH, val=6.0, units='ft')
    indeps.add_output(Aircraft.Fuselage.AVG_HEIGHT, val=5.0, units='inch') 
    indeps.add_output(Aircraft.Fuselage.AVG_WIDTH, val=4.0, units='inch')

    # -----------------------------
    # Geometry & Mass components
    # -----------------------------
    geom_comp = DBFGeom()
    mass_comp = MassSummation()
    fuselage = DBFFuselageMass()
    wing = DBFWingMass()
    htail = DBFHorizontalTailMass()
    vtail = DBFVerticalTailMass()

    # -----------------------------
    # Add geometry subsystem first
    # -----------------------------
    model.add_subsystem('geom', geom_comp, promotes_inputs=[
        Aircraft.Wing.ROOT_CHORD, Aircraft.Wing.SPAN,
        Aircraft.HorizontalTail.ROOT_CHORD, Aircraft.HorizontalTail.SPAN,
        Aircraft.VerticalTail.ROOT_CHORD, Aircraft.VerticalTail.SPAN,
        Aircraft.Fuselage.LENGTH, Aircraft.Fuselage.AVG_HEIGHT, Aircraft.Fuselage.AVG_WIDTH
    ], promotes_outputs=[
        Aircraft.Fuselage.WETTED_AREA,
        Aircraft.Wing.WETTED_AREA,
        Aircraft.HorizontalTail.WETTED_AREA,
        Aircraft.VerticalTail.WETTED_AREA
    ])

    # -----------------------------
    # Add mass subsystems
    # -----------------------------
    model.add_subsystem('fuselage', fuselage, promotes_inputs=[
        Aircraft.Fuselage.WETTED_AREA,
        Aircraft.Fuselage.LENGTH, 
        Aircraft.Fuselage.AVG_HEIGHT, 
        Aircraft.Fuselage.AVG_WIDTH
    ], promotes_outputs=[Aircraft.Fuselage.MASS])

    model.add_subsystem('wing', wing, promotes_inputs=[
        Aircraft.Wing.WETTED_AREA,
        Aircraft.Wing.ROOT_CHORD,
        Aircraft.Wing.SPAN
    ], promotes_outputs=[Aircraft.Wing.MASS])

    model.add_subsystem('htail', htail, promotes_inputs=[
        Aircraft.HorizontalTail.WETTED_AREA,
        Aircraft.HorizontalTail.ROOT_CHORD,
        Aircraft.HorizontalTail.SPAN
    ], promotes_outputs=[Aircraft.HorizontalTail.MASS])

    model.add_subsystem('vtail', vtail, promotes_inputs=[
        Aircraft.VerticalTail.WETTED_AREA,
        Aircraft.VerticalTail.ROOT_CHORD,
        Aircraft.VerticalTail.SPAN
    ], promotes_outputs=[Aircraft.VerticalTail.MASS])

    # -----------------------------
    # Add mass summation component
    # -----------------------------
    model.add_subsystem('mass_sum', mass_comp, promotes_inputs=[
        Aircraft.Wing.MASS,
        Aircraft.Fuselage.MASS,
        Aircraft.HorizontalTail.MASS,
        Aircraft.VerticalTail.MASS
    ], promotes_outputs=[Aircraft.Design.OPERATING_MASS])

    # -----------------------------
    # Manually set airfoil paths
    # -----------------------------
    geom_comp.options[Aircraft.Wing.Dbf.AIRFOIL_PATH] = r"aviary/examples/external_subsystems/dbf_based_mass/n0012-il.csv"
    geom_comp.options[Aircraft.HorizontalTail.Dbf.AIRFOIL_PATH] = r"aviary/examples/external_subsystems/dbf_based_mass/n0012-il.csv"
    geom_comp.options[Aircraft.VerticalTail.Dbf.AIRFOIL_PATH] = r"aviary/examples/external_subsystems/dbf_based_mass/n0012-il.csv"

    # -----------------------------
    # Set airfoil paths for mass components
    # -----------------------------
    wing.options[Aircraft.Wing.Dbf.AIRFOIL_PATH] = r"aviary/examples/external_subsystems/dbf_based_mass/n0012-il.csv"
    htail.options[Aircraft.HorizontalTail.Dbf.AIRFOIL_PATH] = r"aviary/examples/external_subsystems/dbf_based_mass/n0012-il.csv"
    vtail.options[Aircraft.VerticalTail.Dbf.AIRFOIL_PATH] = r"aviary/examples/external_subsystems/dbf_based_mass/n0012-il.csv"


    # -----------------------------
    # Set all required mass/geometry options manually
    # -----------------------------
    # Fuselage
    fuselage.options[Aircraft.Fuselage.Dbf.BULKHEAD_MATERIALS] = ['Balsa']*14 + ['Ply']*6
    fuselage.options[Aircraft.Fuselage.Dbf.BULKHEAD_THICKNESS] = np.array([0.125]*20)
    fuselage.options[Aircraft.Fuselage.Dbf.NUM_SPARS] = 0.5
    fuselage.options[Aircraft.Fuselage.Dbf.BULKHEAD_LIGHTENING_FACTOR] = 0.18
    fuselage.options[Aircraft.Fuselage.Dbf.SHEETING_COVERAGE] = 1.0
    fuselage.options[Aircraft.Fuselage.Dbf.SHEETING_DENSITY] = 160
    fuselage.options[Aircraft.Fuselage.Dbf.SHEETING_LIGHTENING_FACTOR] = 0.3
    fuselage.options[Aircraft.Fuselage.Dbf.SHEETING_THICKNESS] = 0.03125
    fuselage.options[Aircraft.Fuselage.Dbf.GLUE_FACTOR] = 0.08
    fuselage.options[Aircraft.Fuselage.Dbf.STRINGER_DENSITY] = 160
    fuselage.options[Aircraft.Fuselage.Dbf.STRINGER_THICKNESS] = 0.375
    fuselage.options[Aircraft.Fuselage.Dbf.FLOOR_LENGTH] = 2
    fuselage.options[Aircraft.Fuselage.Dbf.FLOOR_DENSITY] = 340
    fuselage.options[Aircraft.Fuselage.Dbf.FLOOR_THICKNESS] = 0.125
    fuselage.options[Aircraft.Fuselage.Dbf.SKIN_DENSITY] = 20
    fuselage.options[Aircraft.Fuselage.Dbf.SPAR_DENSITY] = 2
    fuselage.options[Aircraft.Fuselage.Dbf.SPAR_OUTER_DIAMETER] = 1
    fuselage.options[Aircraft.Fuselage.Dbf.SPAR_WALL_THICKNESS] = 0.0625
    fuselage.options[Aircraft.Fuselage.Dbf.MISC_MASS] = 0.0

    # Wing
    wing.options[Aircraft.Wing.Dbf.RIB_MATERIALS] = ['Balsa']*15 + ['Ply']*5
    wing.options[Aircraft.Wing.Dbf.RIB_THICKNESS] = np.array([0.125]*20)
    wing.options[Aircraft.Wing.Dbf.SHEETING_COVERAGE] = 0.4
    wing.options[Aircraft.Wing.Dbf.SHEETING_DENSITY] = 160
    wing.options[Aircraft.Wing.Dbf.SHEETING_LIGHTENING_FACTOR] = 1.0
    wing.options[Aircraft.Wing.Dbf.SHEETING_THICKNESS] = 0.03125
    wing.options[Aircraft.Wing.Dbf.STRINGER_DENSITY] = 160
    wing.options[Aircraft.Wing.Dbf.STRINGER_THICKNESS] = 0.375
    wing.options[Aircraft.Wing.Dbf.NUM_STRINGERS] = 2.5
    wing.options[Aircraft.Wing.Dbf.GLUE_FACTOR] = 0.15
    wing.options[Aircraft.Wing.Dbf.NUM_SPARS] = 1.1
    wing.options[Aircraft.Wing.Dbf.RIB_LIGHTENING_FACTOR] = 2/3
    wing.options[Aircraft.Wing.Dbf.SKIN_DENSITY] = 20
    wing.options[Aircraft.Wing.Dbf.SPAR_DENSITY] = 2
    wing.options[Aircraft.Wing.Dbf.SPAR_OUTER_DIAMETER] = 1
    wing.options[Aircraft.Wing.Dbf.SPAR_WALL_THICKNESS] = 0.0625
    wing.options[Aircraft.Wing.Dbf.MISC_MASS] = 0.0

    # H-Tail
    htail.options[Aircraft.HorizontalTail.Dbf.RIB_MATERIALS] = ['Balsa']*6 + ['Ply']*2
    htail.options[Aircraft.HorizontalTail.Dbf.RIB_THICKNESS] = np.array([0.125]*8)
    htail.options[Aircraft.HorizontalTail.Dbf.SHEETING_COVERAGE] = 0.7
    htail.options[Aircraft.HorizontalTail.Dbf.SHEETING_DENSITY] = 160
    htail.options[Aircraft.HorizontalTail.Dbf.SHEETING_LIGHTENING_FACTOR] = 1.0
    htail.options[Aircraft.HorizontalTail.Dbf.SHEETING_THICKNESS] = 0.03125
    htail.options[Aircraft.HorizontalTail.Dbf.STRINGER_DENSITY] = 160
    htail.options[Aircraft.HorizontalTail.Dbf.STRINGER_THICKNESS] = 0.375
    htail.options[Aircraft.HorizontalTail.Dbf.NUM_STRINGERS] = 2.5
    htail.options[Aircraft.HorizontalTail.Dbf.GLUE_FACTOR] = 0.05
    htail.options[Aircraft.HorizontalTail.Dbf.NUM_SPARS] = 0
    htail.options[Aircraft.HorizontalTail.Dbf.RIB_LIGHTENING_FACTOR] = 2/3
    htail.options[Aircraft.HorizontalTail.Dbf.SKIN_DENSITY] = 20
    htail.options[Aircraft.HorizontalTail.Dbf.SPAR_DENSITY] = 0
    htail.options[Aircraft.HorizontalTail.Dbf.SPAR_OUTER_DIAMETER] = 0
    htail.options[Aircraft.HorizontalTail.Dbf.SPAR_WALL_THICKNESS] = 0
    htail.options[Aircraft.HorizontalTail.Dbf.MISC_MASS] = 0.0

    # V-Tail
    vtail.options[Aircraft.VerticalTail.Dbf.RIB_MATERIALS] = ['Balsa']*4 + ['Ply']*1
    vtail.options[Aircraft.VerticalTail.Dbf.RIB_THICKNESS] = np.array([0.125]*5)
    vtail.options[Aircraft.VerticalTail.Dbf.SHEETING_COVERAGE] = 0.7
    vtail.options[Aircraft.VerticalTail.Dbf.SHEETING_DENSITY] = 160
    vtail.options[Aircraft.VerticalTail.Dbf.SHEETING_LIGHTENING_FACTOR] = 1.0
    vtail.options[Aircraft.VerticalTail.Dbf.SHEETING_THICKNESS] = 0.03125
    vtail.options[Aircraft.VerticalTail.Dbf.STRINGER_DENSITY] = 160
    vtail.options[Aircraft.VerticalTail.Dbf.STRINGER_THICKNESS] = 0.375
    vtail.options[Aircraft.VerticalTail.Dbf.NUM_STRINGERS] = 2
    vtail.options[Aircraft.VerticalTail.Dbf.GLUE_FACTOR] = 0.05
    vtail.options[Aircraft.VerticalTail.Dbf.NUM_SPARS] = 0
    vtail.options[Aircraft.VerticalTail.Dbf.RIB_LIGHTENING_FACTOR] = 2/3
    vtail.options[Aircraft.VerticalTail.Dbf.SKIN_DENSITY] = 20
    vtail.options[Aircraft.VerticalTail.Dbf.SPAR_DENSITY] = 0
    vtail.options[Aircraft.VerticalTail.Dbf.SPAR_OUTER_DIAMETER] = 0
    vtail.options[Aircraft.VerticalTail.Dbf.SPAR_WALL_THICKNESS] = 0
    vtail.options[Aircraft.VerticalTail.Dbf.MISC_MASS] = 0.0

    # -----------------------------
    # Setup optimizer
    # -----------------------------
    # Add design variables for optimization (now they're available as outputs from indeps)
    model.add_design_var(Aircraft.Wing.ROOT_CHORD, lower=10, upper=30, units='inch')
    model.add_design_var(Aircraft.Wing.SPAN, lower=3, upper=6, units='ft')
    
    # Set objective to minimize total aircraft mass
    model.add_objective(Aircraft.Wing.MASS)

    prob.driver = om.pyOptSparseDriver()
    prob.driver.options['optimizer'] = 'IPOPT'
    
    # Optional: Add some optimizer options for better convergence
    prob.driver.opt_settings['tol'] = 1e-6
    prob.driver.opt_settings['max_iter'] = 100

    prob.setup()
    
    # prob.set_val(Aircraft.Wing.ROOT_CHORD, val=20, units='inch')
    # prob.set_val(Aircraft.Wing.SPAN, val=4.667, units='ft')
    prob.set_val(Aircraft.Wing.CENTER_DISTANCE, val=0.15, units='unitless')
    prob.set_val(Aircraft.Wing.VERTICAL_MOUNT_LOCATION, val=0.75, units='unitless')
    prob.set_val(Aircraft.HorizontalTail.ROOT_CHORD, val=8.75, units='inch')
    prob.set_val(Aircraft.HorizontalTail.SPAN, val=28.0, units='inch')
    prob.set_val(Aircraft.VerticalTail.ROOT_CHORD, val=8.75, units='inch')
    prob.set_val(Aircraft.VerticalTail.SPAN, val=1, units='ft')
    prob.set_val(Aircraft.Fuselage.LENGTH, val=6, units='ft')
    prob.set_val(Aircraft.Fuselage.AVG_WIDTH, val=5, units='inch')
    prob.set_val(Aircraft.Fuselage.AVG_HEIGHT, val=4, units='inch')

    # Debug: Check what the geom component actually receives
    print("Input values set:")
    print(f"Wing root chord: {prob.get_val(Aircraft.Wing.ROOT_CHORD, units='inch')}")
    print(f"Wing span: {prob.get_val(Aircraft.Wing.SPAN, units='ft')}")
    print(f"Fuselage length: {prob.get_val(Aircraft.Fuselage.LENGTH, units='ft')}")
    
    # Test just the geometry component first
    prob.run_model()

    # Run optimizer
    prob.run_driver()
    prob.list_problem_vars()

    # -----------------------------
    # Print results
    # -----------------------------
    print("Wing Root Chord:", prob.get_val(Aircraft.Wing.ROOT_CHORD))
    print("Wing Mass:", prob.get_val(Aircraft.Wing.MASS))