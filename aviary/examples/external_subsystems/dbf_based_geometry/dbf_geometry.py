import os
import numpy as np

try:
    import openvsp as vsp
    from openvsp import (SET_ALL, XS_ROUNDED_RECTANGLE, XS_FOUR_SERIES, EXPORT_STL,
                     EDGE_END_CAP, COMP_GEOM_TXT_TYPE, ROUND_END_CAP, XSEC_CUSTOM)
    HAS_OPENVSP = True
except ImportError:
    HAS_OPENVSP = False

import openmdao.api as om
from openmdao.utils.cs_safe import abs as cs_abs

from aviary.variable_info.functions import add_aviary_input, add_aviary_output
from aviary.variable_info.variables import Aircraft

class DBFGeom(om.ExplicitComponent):

    def initialize(self):
        self.options.declare(Aircraft.Wing.Dbf.AIRFOIL_PATH, types=str, allow_none=False)
        self.options.declare(Aircraft.HorizontalTail.Dbf.AIRFOIL_PATH, types=str, allow_none=False)
        self.options.declare(Aircraft.VerticalTail.Dbf.AIRFOIL_PATH, types=str, allow_none=False)


    def setup(self):
        add_aviary_input(self, Aircraft.Wing.ROOT_CHORD, units='m')
        add_aviary_input(self, Aircraft.Wing.SPAN, units='m')
        add_aviary_input(self, Aircraft.Wing.CENTER_DISTANCE, units='unitless')
        add_aviary_input(self, Aircraft.Wing.VERTICAL_MOUNT_LOCATION, units="unitless")
        add_aviary_input(self, Aircraft.HorizontalTail.ROOT_CHORD, units='m')
        add_aviary_input(self, Aircraft.HorizontalTail.SPAN, units='m')
        add_aviary_input(self, Aircraft.VerticalTail.ROOT_CHORD, units='m')
        add_aviary_input(self, Aircraft.VerticalTail.SPAN, units = 'm')
        add_aviary_input(self, Aircraft.Fuselage.LENGTH, units='m')
        add_aviary_input(self, Aircraft.Fuselage.AVG_WIDTH, units='m')
        add_aviary_input(self, Aircraft.Fuselage.AVG_HEIGHT, units='m')

        add_aviary_output(self, Aircraft.Wing.WETTED_AREA, units='m**2')
        add_aviary_output(self, Aircraft.HorizontalTail.WETTED_AREA, units='m**2')
        add_aviary_output(self, Aircraft.VerticalTail.WETTED_AREA, units='m**2')
        add_aviary_output(self, Aircraft.Wing.AREA, units='m**2')
        add_aviary_output(self, Aircraft.HorizontalTail.AREA, units='m**2')
        add_aviary_output(self, Aircraft.VerticalTail.AREA, units='m**2')
        add_aviary_output(self, Aircraft.Fuselage.WETTED_AREA, units='m**2')

    def setup_partials(self):
        self.declare_partials(
            of=Aircraft.Wing.WETTED_AREA,
            wrt=[
                Aircraft.Wing.SPAN,
                Aircraft.Wing.ROOT_CHORD,
            ],
            method='fd',
        )
        self.declare_partials(
            of=Aircraft.HorizontalTail.WETTED_AREA,
            wrt=[
                Aircraft.HorizontalTail.SPAN,
                Aircraft.HorizontalTail.ROOT_CHORD,
            ],
            method='fd',
        )
        self.declare_partials(
            of=Aircraft.VerticalTail.WETTED_AREA,
            wrt=[
                Aircraft.VerticalTail.SPAN,
                Aircraft.VerticalTail.ROOT_CHORD,
            ],
            method='fd',
        )
        self.declare_partials(
            of=Aircraft.Wing.AREA,
            wrt=[
                Aircraft.Wing.SPAN,
                Aircraft.Wing.ROOT_CHORD,
            ],
            method='fd',
        )
        self.declare_partials(
            of=Aircraft.HorizontalTail.AREA,
            wrt=[
                Aircraft.HorizontalTail.SPAN,
                Aircraft.HorizontalTail.ROOT_CHORD,
            ],
            method='fd',
        )
        self.declare_partials(
            of=Aircraft.VerticalTail.AREA,
            wrt=[
                Aircraft.VerticalTail.SPAN,
                Aircraft.VerticalTail.ROOT_CHORD,
            ],
            method='fd',
        )
        self.declare_partials(
            of=Aircraft.Fuselage.WETTED_AREA,
            wrt=[
                Aircraft.Fuselage.LENGTH,
                Aircraft.Fuselage.AVG_WIDTH,
                Aircraft.Fuselage.AVG_HEIGHT,
            ],
            method='fd',
        )

    def vsp_load_airfoil_csv(self, file_path, delimiter=',', header=False):
            if not os.path.exists(file_path):
                raise FileNotFoundError(f"Airfoil CSV file '{file_path}' not found.")

            skip = 1 if header else 0
            data = np.loadtxt(file_path, delimiter=delimiter, skiprows=skip)

            if data.shape[1] < 2:
                raise ValueError('CSV must contain at least two columns for x and y coordinates.')

            x = data[:, 0]
            y = data[:, 1]

            x_min = np.min(x)
            x_max = np.max(x)
            chord_length = x_max - x_min

            if chord_length <= 0:
                raise ValueError('Invalid airfoil: chord length must be > 0.')

            x_norm = (x - x_min) / chord_length
            y_norm = y / chord_length

            # === Split into upper and lower surfaces ===
            le_index = np.argmin(x_norm)  # leading edge
            upper_coords = np.array(list(zip(x_norm[:le_index+1], y_norm[:le_index+1])))
            lower_coords = np.array(list(zip(x_norm[le_index:], y_norm[le_index:])))

            # Reverse upper to go LE → TE
            upper_coords = upper_coords[::-1]

            # === Create vec3d points ===
            upper_pnts = [vsp.vec3d(x, y, 0.0) for x, y in upper_coords]
            lower_pnts = [vsp.vec3d(x, y, 0.0) for x, y in lower_coords]

            return upper_pnts, lower_pnts
    
    def load_airfoil_csv(self, file_path, delimiter=',', header=False):
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"Airfoil CSV file '{file_path}' not found.")

        skip = 1 if header else 0
        data = np.loadtxt(file_path, delimiter=delimiter, skiprows=skip)

        if data.shape[1] < 2:
            raise ValueError('CSV must contain at least two columns for x and y coordinates.')

        x = data[:, 0]
        y = data[:, 1]

        x_min = np.min(x)
        x_max = np.max(x)
        chord_length = x_max - x_min

        if chord_length <= 0:
            raise ValueError('Invalid airfoil: chord length must be > 0.')

        x_normalized = (x - x_min) / chord_length
        y_normalized = y / chord_length

        return x_normalized, y_normalized

    def shoelace_area(self, x, y):
        return 0.5 * cs_abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))

    def compute(self, inputs, outputs):
        wing_chord = inputs[Aircraft.Wing.ROOT_CHORD][0]
        wing_span = inputs[Aircraft.Wing.SPAN][0]
        wing_mount_x = inputs[Aircraft.Wing.CENTER_DISTANCE][0]
        wing_mount_y = inputs[Aircraft.Wing.VERTICAL_MOUNT_LOCATION][0]
        htail_chord = inputs[Aircraft.HorizontalTail.ROOT_CHORD][0]
        htail_span = inputs[Aircraft.HorizontalTail.SPAN][0]
        vtail_chord = inputs[Aircraft.VerticalTail.ROOT_CHORD][0]
        vtail_span = inputs[Aircraft.VerticalTail.SPAN][0]
        fuse_len = inputs[Aircraft.Fuselage.LENGTH][0]
        fuse_width = inputs[Aircraft.Fuselage.AVG_WIDTH][0]
        fuse_height = inputs[Aircraft.Fuselage.AVG_HEIGHT][0]
        wing_airfoil_data_file = self.options[Aircraft.Wing.Dbf.AIRFOIL_PATH]  # stays string key
        htail_airfoil_data_file = self.options[Aircraft.HorizontalTail.Dbf.AIRFOIL_PATH]  # stays string key
        vtail_airfoil_data_file = self.options[Aircraft.VerticalTail.Dbf.AIRFOIL_PATH]  # stays string key

        outputs[Aircraft.Wing.AREA] = wing_span * wing_chord
        outputs[Aircraft.HorizontalTail.AREA] = htail_span * htail_chord        
        outputs[Aircraft.VerticalTail.AREA] = vtail_span * vtail_chord


        HAS_OPENVSP = False

        if HAS_OPENVSP:

            wing_chord *= 10
            wing_span *= 10
            htail_chord *= 10
            htail_span *= 10
            vtail_chord *= 10
            vtail_span *= 10
            fuse_len *= 10
            fuse_width *= 10
            fuse_height *= 10

            ### VSP THINGS ###
            # Add Fuselage Geom
            fuseid = vsp.AddGeom("FUSELAGE", "")
            vsp.SetParmVal(fuseid, "Length", "Design", fuse_len)  # Set desired fuselage length

            # Loop over all fuselage sections
            num_sections = vsp.GetNumXSecSurfs(fuseid)
            for i in range(num_sections):

                xsec_surf = vsp.GetXSecSurf(fuseid, i)
                num_xsecs = vsp.GetNumXSec(xsec_surf)

                for j in range(num_xsecs):

                    if j == (num_xsecs - 1):
                        xsec = vsp.GetXSec(xsec_surf, j)
                        if vsp.GetXSecShape(xsec) != XS_ROUNDED_RECTANGLE:
                            vsp.ChangeXSecShape(xsec_surf, j, XS_ROUNDED_RECTANGLE)
                        
                        xsec = vsp.GetXSec(xsec_surf, j)
                        zid = vsp.GetXSecParm(xsec, "ZLocPercent")
                        vsp.SetParmVal(zid, (fuse_height - 0.5) / fuse_len / 2)
                        wid = vsp.GetXSecParm(xsec, "RoundedRect_Width")
                        vsp.SetParmVal(wid, 0.5)
                        hid = vsp.GetXSecParm(xsec, "RoundedRect_Height")
                        vsp.SetParmVal(hid, 0.5)
                        taid = vsp.GetXSecParm(xsec, "TopLAngle")
                        vsp.SetParmVal(taid, 0.0)
                        raid = vsp.GetXSecParm(xsec, "RightLAngle")
                        vsp.SetParmVal(raid, 0.0)
                        tsid = vsp.GetXSecParm(xsec, "TopLStrength")
                        vsp.SetParmVal(tsid, 0.05)
                        rsid = vsp.GetXSecParm(xsec, "RightLStrength")
                        vsp.SetParmVal(rsid, 0.05)
                    
                    elif j == 0:
                        xsec = vsp.GetXSec(xsec_surf, j)
                        if vsp.GetXSecShape(xsec) != XS_ROUNDED_RECTANGLE:
                            vsp.ChangeXSecShape(xsec_surf, j, XS_ROUNDED_RECTANGLE)
                            
                        xsec = vsp.GetXSec(xsec_surf, j)
                        wid = vsp.GetXSecParm(xsec, "RoundedRect_Width")
                        vsp.SetParmVal(wid, fuse_width / 2)
                        hid = vsp.GetXSecParm(xsec, "RoundedRect_Height")
                        vsp.SetParmVal(hid, fuse_height / 2)
                        taid = vsp.GetXSecParm(xsec, "TopLAngle")
                        vsp.SetParmVal(taid, 10.0)
                        raid = vsp.GetXSecParm(xsec, "RightLAngle")
                        vsp.SetParmVal(raid, 10.0)
                        tsid = vsp.GetXSecParm(xsec, "TopLStrength")
                        vsp.SetParmVal(tsid, 1.0)
                        rsid = vsp.GetXSecParm(xsec, "RightLStrength")
                        vsp.SetParmVal(rsid, 1.0)

                    else:
                        xsec = vsp.GetXSec(xsec_surf, j)
                        if vsp.GetXSecShape(xsec) != XS_ROUNDED_RECTANGLE:
                            vsp.ChangeXSecShape(xsec_surf, j, XS_ROUNDED_RECTANGLE)
                            
                        xsec = vsp.GetXSec(xsec_surf, j)
                        wid = vsp.GetXSecParm(xsec, "RoundedRect_Width")
                        vsp.SetParmVal(wid, fuse_width)
                        hid = vsp.GetXSecParm(xsec, "RoundedRect_Height")
                        vsp.SetParmVal(hid, fuse_height)

                fcap = vsp.GetParm(fuseid, "CapUMinOption", "EndCap")
                vsp.SetParmVal(fcap, ROUND_END_CAP)
                fcap_length = vsp.GetParm(fuseid, "CapUMinLength", "EndCap")
                vsp.SetParmVal(fcap_length, 0.015 * fuse_len)
                ecap = vsp.GetParm(fuseid, "CapUMaxOption", "EndCap")
                vsp.SetParmVal(ecap, EDGE_END_CAP)
                ecap_length = vsp.GetParm(fuseid, "CapUMaxLength", "EndCap")
                vsp.SetParmVal(ecap_length, 0.05)


            vsp.Update()

            ### Wing ###
            wid = vsp.AddGeom( "WING", "" ) # Add Wing
            # Set wing parameters to remove taper and sweep
            vsp.SetParmVal(wid, "Root_Chord", "XSec_1", wing_chord)   # Root chord length
            vsp.SetParmVal(wid, "Tip_Chord", "XSec_1", wing_chord)    # Tip chord = Root chord → no taper
            vsp.SetParmVal(wid, "Sweep", "XSec_1", 0.0)        # No sweep
            vsp.SetParmVal(wid, "Dihedral", "XSec_1", 0.0)     # (optional) No dihedral
            vsp.SetParmVal(wid, "Span", "XSec_1", wing_span)        # Set desired span
            vsp.SetParmVal(wid, "Twist", "XSec_1", 0.0)        # (optional) No twist
            vsp.SetParmVal(wid, "X_Rel_Location", "XForm", wing_mount_x * fuse_len)
            vsp.SetParmVal(wid, "Z_Rel_Location", "XForm", wing_mount_y * fuse_height - fuse_height)

            xsec_surf_id = vsp.GetXSecSurf(wid, 0)

            upper_pnts, lower_pnts = self.vsp_load_airfoil_csv(
                wing_airfoil_data_file, 
                delimiter=",", 
                header=True)
            
            # Loop over all cross sections
            for xsec_index in range(vsp.GetNumXSec(xsec_surf_id)):
                vsp.ChangeXSecShape(xsec_surf_id, xsec_index, vsp.XS_FILE_AIRFOIL)
                xsec_id = vsp.GetXSec(xsec_surf_id, xsec_index)
                vsp.SetAirfoilPnts(xsec_id, upper_pnts, lower_pnts)


            ### HTAIL ###
            htail_id = vsp.AddGeom( "WING", "" ) # Add Wing
            
            vsp.SetGeomName(htail_id, "HtailGeom")

            # Set wing parameters to remove taper and sweep
            vsp.SetParmVal(htail_id, "Root_Chord", "XSec_1", htail_chord)   # Root chord length
            vsp.SetParmVal(htail_id, "Tip_Chord", "XSec_1", htail_chord)    # Tip chord = Root chord → no taper
            vsp.SetParmVal(htail_id, "Sweep", "XSec_1", 0.0)        # No sweep
            vsp.SetParmVal(htail_id, "Dihedral", "XSec_1", 0.0)     # (optional) No dihedral
            vsp.SetParmVal(htail_id, "Span", "XSec_1", htail_span)         # Set desired span
            vsp.SetParmVal(htail_id, "Twist", "XSec_1", 0.0)        # (optional) No twist
            vsp.SetParmVal(htail_id, "X_Rel_Location", "XForm", fuse_len - htail_chord)
            vsp.SetParmVal(htail_id, "Z_Rel_Location", "XForm", fuse_height / 2 - 0.25)

            xsec_surf_id = vsp.GetXSecSurf(htail_id, 0)

            upper_pnts, lower_pnts = self.vsp_load_airfoil_csv(
                htail_airfoil_data_file, 
                delimiter=",", 
                header=True)
            
            # Loop over all cross sections
            for xsec_index in range(vsp.GetNumXSec(xsec_surf_id)):
                vsp.ChangeXSecShape(xsec_surf_id, xsec_index, vsp.XS_FILE_AIRFOIL)
                xsec_id = vsp.GetXSec(xsec_surf_id, xsec_index)
                vsp.SetAirfoilPnts(xsec_id, upper_pnts, lower_pnts)

            ### VTAIL ###
            vtail_id = vsp.AddGeom("WING", "")

            vsp.SetGeomName(vtail_id, "VtailGeom")

            # Set vertical tail parameters
            vsp.SetParmVal(vtail_id, "Root_Chord", "XSec_1", vtail_chord)
            vsp.SetParmVal(vtail_id, "Tip_Chord", "XSec_1", vtail_chord)
            vsp.SetParmVal(vtail_id, "Span", "XSec_1", vtail_span)            # Acts as "height" here
            vsp.SetParmVal(vtail_id, "Sweep", "XSec_1", 0) # np.rad2deg(np.arctan(vtail_chord/2/vtail_span)))
            vsp.SetParmVal(vtail_id, "Twist", "XSec_1", 0.0)

            # Move it to the rear of the fuselage
            vsp.SetParmVal(vtail_id, "X_Rel_Location", "XForm", fuse_len - vtail_chord)
            vsp.SetParmVal(vtail_id, "Z_Rel_Location", "XForm", fuse_height / 2 - 0.05)

            # Rotate to make it vertical (pitch = 90 degrees)
            vsp.SetParmVal(vtail_id, "X_Rel_Rotation", "XForm", 90.0)
            vsp.SetParmVal(vtail_id, "Sym_Planar_Flag", "Sym", 0)

            xsec_surf_id = vsp.GetXSecSurf(vtail_id, 0)

            upper_pnts, lower_pnts = self.vsp_load_airfoil_csv(
                vtail_airfoil_data_file, 
                delimiter=",", 
                header=True)
            
            # Loop over all cross sections
            for xsec_index in range(vsp.GetNumXSec(xsec_surf_id)):
                vsp.ChangeXSecShape(xsec_surf_id, xsec_index, vsp.XS_FILE_AIRFOIL)
                xsec_id = vsp.GetXSec(xsec_surf_id, xsec_index)
                vsp.SetAirfoilPnts(xsec_id, upper_pnts, lower_pnts)

            # Save file
            fname = "dbf.vsp3"

            vsp.SetVSP3FileName( fname )

            vsp.Update()

            mesh_id = vsp.ComputeCompGeom(vsp.SET_ALL, False, 0) # COMP_GEOM_TXT_TYPE)
            comp_geom_res_id = vsp.FindLatestResultsID("Comp_Geom")
            areas = vsp.GetDoubleResults(comp_geom_res_id, "Wet_Area")

            # print(areas)

            fuse_wet_area = areas[0] / 100
            wing_wet_area = areas[1] / 100
            htail_wet_area = areas[2] / 100
            vtail_wet_area = areas[3] / 100

            outputs[Aircraft.Fuselage.WETTED_AREA] = fuse_wet_area
            outputs[Aircraft.Wing.WETTED_AREA] = wing_wet_area
            outputs[Aircraft.HorizontalTail.WETTED_AREA] = htail_wet_area
            outputs[Aircraft.VerticalTail.WETTED_AREA] = vtail_wet_area

            # #==== Save Vehicle to File ====//
            # print( "\tSaving vehicle file to: ", fname )
            # vsp.WriteVSPFile( vsp.GetVSPFileName(), SET_ALL )
            # vsp.ExportFile( "dbf.stl", SET_ALL, EXPORT_STL)

            #==== Reset Geometry ====//
            print( "--->Resetting VSP model to blank slate\n" )

            vsp.ClearVSPModel()

        else:
            ### FUSE ###
            fuse_wet_area = 2 * (fuse_len * fuse_height + fuse_height * fuse_width + fuse_width * fuse_len)

            ### WING ###
            x_norm, y_norm = self.load_airfoil_csv(wing_airfoil_data_file, header=True)
            # Calculate total length along airfoil curve (all points in order)
            length_norm = np.sum(np.sqrt(np.diff(x_norm)**2 + np.diff(y_norm)**2))
            # Convert to physical length
            length = length_norm * wing_chord
            wing_cs_area = self.shoelace_area(x_norm,y_norm) * wing_chord**2
            # Wetted area approx = length of curve * span
            wing_wet_area = length * wing_span + 2 * wing_cs_area

            ### HTAIL ###
            x_norm, y_norm = self.load_airfoil_csv(htail_airfoil_data_file, header=True)
            # Calculate total length along airfoil curve (all points in order)
            length_norm = np.sum(np.sqrt(np.diff(x_norm)**2 + np.diff(y_norm)**2))
            # Convert to physical length
            length = length_norm * htail_chord
            htail_cs_area = self.shoelace_area(x_norm,y_norm) * htail_chord**2
            # Wetted area approx = length of curve * span
            htail_wet_area = length * htail_span + 2 * htail_cs_area

            ### VTAIL ###
            x_norm, y_norm = self.load_airfoil_csv(vtail_airfoil_data_file, header=True)
            # Calculate total length along airfoil curve (all points in order)
            length_norm = np.sum(np.sqrt(np.diff(x_norm)**2 + np.diff(y_norm)**2))
            # Convert to physical length
            length = length_norm * vtail_chord
            vtail_cs_area = self.shoelace_area(x_norm,y_norm) * vtail_chord**2
            # Wetted area approx = length of curve * span
            vtail_wet_area = length * vtail_span + vtail_cs_area

            outputs[Aircraft.Fuselage.WETTED_AREA] = fuse_wet_area
            outputs[Aircraft.Wing.WETTED_AREA] = wing_wet_area
            outputs[Aircraft.HorizontalTail.WETTED_AREA] = htail_wet_area
            outputs[Aircraft.VerticalTail.WETTED_AREA] = vtail_wet_area
    

if __name__ == "__main__":
    prob = om.Problem()

    prob.model.add_subsystem(
        'dbf_geom', DBFGeom(), promotes_inputs=['*'], promotes_outputs=['*']
    )

    paths = prob.model.dbf_geom
    paths.options[Aircraft.Wing.Dbf.AIRFOIL_PATH] = (
        r'aviary\examples\external_subsystems\dbf_based_mass\mh84-il.csv'
    )
    paths.options[Aircraft.HorizontalTail.Dbf.AIRFOIL_PATH] = (
        r'aviary\examples\external_subsystems\dbf_based_mass\n0012-il.csv'
    )
    paths.options[Aircraft.VerticalTail.Dbf.AIRFOIL_PATH] = (
        r'aviary\examples\external_subsystems\dbf_based_mass\n0012-il.csv'
    )

    prob.setup()

    # Set values for mesh-driving variables
    prob.set_val(Aircraft.Wing.ROOT_CHORD, val=20, units='inch')
    prob.set_val(Aircraft.Wing.SPAN, val=4.667, units='ft')
    prob.set_val(Aircraft.Wing.CENTER_DISTANCE, val=0.15, units='unitless')
    prob.set_val(Aircraft.Wing.VERTICAL_MOUNT_LOCATION, val=0.75, units='unitless')
    prob.set_val(Aircraft.HorizontalTail.ROOT_CHORD, val=8.75, units='inch')
    prob.set_val(Aircraft.HorizontalTail.SPAN, val=28.0, units='inch')
    prob.set_val(Aircraft.VerticalTail.ROOT_CHORD, val=8.75, units='inch')
    prob.set_val(Aircraft.VerticalTail.SPAN, val=1, units='ft')
    prob.set_val(Aircraft.Fuselage.LENGTH, val=6, units='ft')
    prob.set_val(Aircraft.Fuselage.AVG_WIDTH, val=5, units='inch')
    prob.set_val(Aircraft.Fuselage.AVG_HEIGHT, val=4, units='inch')

    prob.run_model()

    fuse_wet_area = prob.get_val(Aircraft.Fuselage.WETTED_AREA, units='inch**2')
    wing_wet_area = prob.get_val(Aircraft.Wing.WETTED_AREA, units='inch**2')
    htail_wet_area = prob.get_val(Aircraft.HorizontalTail.WETTED_AREA, units='inch**2')
    vtail_wet_area = prob.get_val(Aircraft.VerticalTail.WETTED_AREA, units='inch**2')

    print(f"Fuselage Wetted Area: {float(fuse_wet_area):.3f} inch**2")
    print(f"Wing Wetted Area: {float(wing_wet_area):.3f} inch**2")
    print(f"HTail Wetted Area: {float(htail_wet_area):.3f} inch**2")
    print(f"VTail Wetted Area: {float(vtail_wet_area):.3f} inch**2")