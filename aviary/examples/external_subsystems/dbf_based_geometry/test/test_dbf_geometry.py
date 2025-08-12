import unittest
import numpy as np
import os
import tempfile

import openmdao.api as om

from aviary.variable_info.variables import Aircraft
from openmdao.utils.assert_utils import assert_near_equal, assert_check_partials

# Import your DBFGeom class (adjust the import path as needed)
from aviary.examples.external_subsystems.dbf_based_geometry.dbf_geometry import DBFGeom 


class TestDBFGeom(unittest.TestCase):
    def setUp(self):
        self.prob = om.Problem()
        self.dbf_geom = DBFGeom()

        self.prob.model.add_subsystem(
            'dbf_geom', self.dbf_geom, promotes_inputs=['*'], promotes_outputs=['*']
        )

        # Create temporary airfoil CSV files for testing
        self.temp_files = []
        self.wing_airfoil_path = self._create_test_airfoil_csv("wing_test.csv")
        self.htail_airfoil_path = self._create_test_airfoil_csv("htail_test.csv")
        self.vtail_airfoil_path = self._create_test_airfoil_csv("vtail_test.csv")

        # Set airfoil paths
        self.dbf_geom.options[Aircraft.Wing.Dbf.AIRFOIL_PATH] = self.wing_airfoil_path
        self.dbf_geom.options[Aircraft.HorizontalTail.Dbf.AIRFOIL_PATH] = self.htail_airfoil_path
        self.dbf_geom.options[Aircraft.VerticalTail.Dbf.AIRFOIL_PATH] = self.vtail_airfoil_path

        self.prob.setup(force_alloc_complex=True)

        # Set input values similar to your main section
        self.prob.set_val(Aircraft.Wing.ROOT_CHORD, val=20, units='inch')
        self.prob.set_val(Aircraft.Wing.SPAN, val=4.667, units='ft')
        self.prob.set_val(Aircraft.Wing.CENTER_DISTANCE, val=0.15, units='unitless')
        self.prob.set_val(Aircraft.Wing.VERTICAL_MOUNT_LOCATION, val=0.75, units='unitless')
        self.prob.set_val(Aircraft.HorizontalTail.ROOT_CHORD, val=8.75, units='inch')
        self.prob.set_val(Aircraft.HorizontalTail.SPAN, val=28.0, units='inch')
        self.prob.set_val(Aircraft.VerticalTail.ROOT_CHORD, val=8.75, units='inch')
        self.prob.set_val(Aircraft.VerticalTail.SPAN, val=1, units='ft')
        self.prob.set_val(Aircraft.Fuselage.LENGTH, val=6, units='ft')
        self.prob.set_val(Aircraft.Fuselage.AVG_WIDTH, val=5, units='inch')
        self.prob.set_val(Aircraft.Fuselage.AVG_HEIGHT, val=4, units='inch')

    def tearDown(self):
        # Clean up temporary files
        for temp_file in self.temp_files:
            if os.path.exists(temp_file):
                os.remove(temp_file)

    def _create_test_airfoil_csv(self, filename):
        """Create a simple test airfoil CSV file"""
        # Create a simple symmetric airfoil profile
        x_coords = np.array([1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.0,
                            0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0])
        y_coords = np.array([0.0, 0.02, 0.035, 0.045, 0.05, 0.052, 0.05, 0.045, 0.035, 0.02, 0.0,
                            -0.02, -0.035, -0.045, -0.05, -0.052, -0.05, -0.045, -0.035, -0.02, 0.0])
        
        # Create temporary file
        temp_dir = tempfile.gettempdir()
        filepath = os.path.join(temp_dir, filename)
        
        # Write CSV with header
        with open(filepath, 'w') as f:
            f.write("x,y\n")
            for x, y in zip(x_coords, y_coords):
                f.write(f"{x:.6f},{y:.6f}\n")
        
        self.temp_files.append(filepath)
        return filepath

    def test_wetted_area_outputs(self):
        """Test that the component produces reasonable wetted area outputs"""
        self.prob.run_model()

        fuse_wet_area = self.prob.get_val(Aircraft.Fuselage.WETTED_AREA, units='inch**2')
        wing_wet_area = self.prob.get_val(Aircraft.Wing.WETTED_AREA, units='inch**2')
        htail_wet_area = self.prob.get_val(Aircraft.HorizontalTail.WETTED_AREA, units='inch**2')
        vtail_wet_area = self.prob.get_val(Aircraft.VerticalTail.WETTED_AREA, units='inch**2')

        print(f"Fuselage Wetted Area: {float(fuse_wet_area):.3f} inch**2")
        print(f"Wing Wetted Area: {float(wing_wet_area):.3f} inch**2")
        print(f"HTail Wetted Area: {float(htail_wet_area):.3f} inch**2")
        print(f"VTail Wetted Area: {float(vtail_wet_area):.3f} inch**2")

        # Basic sanity checks - all areas should be positive
        self.assertGreater(fuse_wet_area, 0, "Fuselage wetted area should be positive")
        self.assertGreater(wing_wet_area, 0, "Wing wetted area should be positive")
        self.assertGreater(htail_wet_area, 0, "HTail wetted area should be positive")
        self.assertGreater(vtail_wet_area, 0, "VTail wetted area should be positive")

    def test_wing_partials(self):
        """Test partial derivatives for wing wetted area"""
        self.prob.run_model()
        
        # Test partials specifically for wing
        partials_data = self.prob.check_partials(
            includes=[f'dbf_geom.{Aircraft.Wing.WETTED_AREA}'],
            compact_print=True, 
            method='cs'
        )
        assert_check_partials(partials_data, atol=1e-6, rtol=1e-6)

    def test_htail_partials(self):
        """Test partial derivatives for horizontal tail wetted area"""
        self.prob.run_model()
        
        # Test partials specifically for horizontal tail
        partials_data = self.prob.check_partials(
            includes=[f'dbf_geom.{Aircraft.HorizontalTail.WETTED_AREA}'],
            compact_print=True, 
            method='cs'
        )
        assert_check_partials(partials_data, atol=1e-6, rtol=1e-6)

    def test_vtail_partials(self):
        """Test partial derivatives for vertical tail wetted area"""
        self.prob.run_model()
        
        # Test partials specifically for vertical tail
        partials_data = self.prob.check_partials(
            includes=[f'dbf_geom.{Aircraft.VerticalTail.WETTED_AREA}'],
            compact_print=True, 
            method='cs'
        )
        assert_check_partials(partials_data, atol=1e-6, rtol=1e-6)

    def test_fuselage_partials(self):
        """Test partial derivatives for fuselage wetted area"""
        self.prob.run_model()
        
        # Test partials specifically for fuselage
        partials_data = self.prob.check_partials(
            includes=[f'dbf_geom.{Aircraft.Fuselage.WETTED_AREA}'],
            compact_print=True, 
            method='cs'
        )
        assert_check_partials(partials_data, atol=1e-6, rtol=1e-6)

    def test_all_partials(self):
        """Test all partial derivatives at once"""
        self.prob.run_model()
        
        partials_data = self.prob.check_partials(compact_print=True, method='cs')
        assert_check_partials(partials_data, atol=1e-6, rtol=1e-6)

if __name__ == '__main__':
    unittest.main()