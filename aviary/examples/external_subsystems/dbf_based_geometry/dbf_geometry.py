import os
import numpy as np

import openvsp as vsp 
from openvsp import (SET_ALL, XS_ROUNDED_RECTANGLE, XS_FOUR_SERIES, EXPORT_STL,
                     EDGE_END_CAP, COMP_GEOM_TXT_TYPE, ROUND_END_CAP, XSEC_CUSTOM)
import openmdao.api as om

from aviary.examples.external_subsystems.dbf_based_mass.materials_database import materials
from aviary.utils.utils import wrapped_convert_units
from aviary.variable_info.functions import add_aviary_input, add_aviary_output
from aviary.variable_info.variables import Aircraft
from aviary.variable_info.variable_meta_data import _MetaData

class DBFGeom(om.ExplicitComponent):

    def setup(self):
        add_aviary_input(self, Aircraft.Wing.ROOT_CHORD, units='inch')
        add_aviary_input(self, Aircraft.Wing.SPAN, units='inch')
        add_aviary_input(self, Aircraft.HorizontalTail.ROOT_CHORD, units='inch')
        add_aviary_input(self, Aircraft.HorizontalTail.SPAN, units='inch')
        add_aviary_input(self, Aircraft.VerticalTail.ROOT_CHORD, units='inch')
        add_aviary_input(self, Aircraft.VerticalTail.SPAN, units = 'inch')
        add_aviary_input(self, Aircraft.Fuselage.LENGTH, units='inch')
        add_aviary_input(self, Aircraft.Fuselage.AVG_WIDTH, units='inch')
        add_aviary_input(self, Aircraft.Fuselage.AVG_HEIGHT, units='inch')

        add_aviary_output(self, Aircraft.Wing.WETTED_AREA, units='inch**2')
        add_aviary_output(self, Aircraft.HorizontalTail.WETTED_AREA, units='inch**2')
        add_aviary_output(self, Aircraft.VerticalTail.WETTED_AREA, units='inch**2')
        add_aviary_output(self, Aircraft.Fuselage.WETTED_AREA, units='inch**2')

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


    def compute(self, inputs, outputs):
        wing_chord = inputs[Aircraft.Wing.ROOT_CHORD][0]
        wing_span = inputs[Aircraft.Wing.SPAN][0]
        htail_chord = inputs[Aircraft.HorizontalTail.ROOT_CHORD][0]
        htail_span = inputs[Aircraft.HorizontalTail.SPAN][0]
        vtail_chord = inputs[Aircraft.VerticalTail.ROOT_CHORD][0]
        vtail_span = inputs[Aircraft.VerticalTail.SPAN][0]
        fuse_len = inputs[Aircraft.Fuselage.LENGTH][0]
        fuse_width = inputs[Aircraft.Fuselage.AVG_WIDTH][0]
        fuse_height = inputs[Aircraft.Fuselage.AVG_HEIGHT][0]

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
                    vsp.SetParmVal(wid, 5.0/2)
                    hid = vsp.GetXSecParm(xsec, "RoundedRect_Height")
                    vsp.SetParmVal(hid, 4.0/2)
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
        vsp.SetParmVal(wid, "X_Rel_Location", "XForm", 9.0)
        vsp.SetParmVal(wid, "Z_Rel_Location", "XForm", - 1 * fuse_height + 0.2 * wing_chord)

        # After creating the wing and setting span, chord, etc.
        # xsec = vsp.GetXSec(vsp.GetXSecSurf(wid, 0), 1)     # Get the actual XSec object
        # tc_parm = vsp.GetXSecParm(xsec, "ThickChord")      # Get the t/c parameter
        # vsp.SetParmVal(tc_parm, 0.2)                       # Set thickness-to-chord ratio (e.g., 15%)
        # xsec = vsp.GetXSec(vsp.GetXSecSurf(wid, 0), 0)     # Get the actual XSec object
        # tc_parm = vsp.GetXSecParm(xsec, "ThickChord")      # Get the t/c parameter
        # vsp.SetParmVal(tc_parm, 0.2)                       # Set thickness-to-chord ratio (e.g., 15%)

        xsec_surf = vsp.GetXSecSurf(wid, 0)
        vsp.ChangeXSecShape(xsec, 0, XSEC_CUSTOM)
        vsp.ChangeXSecShape(xsec, 1, XSEC_CUSTOM)
        vsp.SetEditXSecFixedUVec

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

        # After creating the wing and setting span, chord, etc.
        xsec = vsp.GetXSec(vsp.GetXSecSurf(htail_id, 0), 1)     # Get the actual XSec object
        tc_parm = vsp.GetXSecParm(xsec, "ThickChord")      # Get the t/c parameter
        vsp.SetParmVal(tc_parm, 0.2)                      # Set thickness-to-chord ratio (e.g., 15%)
        xsec = vsp.GetXSec(vsp.GetXSecSurf(htail_id, 0), 0)     # Get the actual XSec object
        tc_parm = vsp.GetXSecParm(xsec, "ThickChord")      # Get the t/c parameter
        vsp.SetParmVal(tc_parm, 0.2)                      # Set thickness-to-chord ratio (e.g., 15%)

        ### VTAIL ###
        vtail_id = vsp.AddGeom("WING", "")

        vsp.SetGeomName(vtail_id, "VtailGeom")

        # Set vertical tail parameters
        vsp.SetParmVal(vtail_id, "Root_Chord", "XSec_1", vtail_chord)
        vsp.SetParmVal(vtail_id, "Tip_Chord", "XSec_1", vtail_chord/2)
        vsp.SetParmVal(vtail_id, "Span", "XSec_1", vtail_span)            # Acts as "height" here
        vsp.SetParmVal(vtail_id, "Sweep", "XSec_1", np.rad2deg(np.arctan(vtail_chord/2/vtail_span)))
        vsp.SetParmVal(vtail_id, "Twist", "XSec_1", 0.0)

        # Move it to the rear of the fuselage
        vsp.SetParmVal(vtail_id, "X_Rel_Location", "XForm", fuse_len - vtail_chord)
        vsp.SetParmVal(vtail_id, "Z_Rel_Location", "XForm", fuse_height / 2 - 0.05)

        # Rotate to make it vertical (pitch = 90 degrees)
        vsp.SetParmVal(vtail_id, "X_Rel_Rotation", "XForm", 90.0)
        vsp.SetParmVal(vtail_id, "Sym_Planar_Flag", "Sym", 0)

        # Set airfoil shape and thickness/chord ratio
        xsec = vsp.GetXSec(vsp.GetXSecSurf(vtail_id, 0), 1)
        tc_parm = vsp.GetXSecParm(xsec, "ThickChord")
        vsp.SetParmVal(tc_parm, 0.12)
        xsec = vsp.GetXSec(vsp.GetXSecSurf(vtail_id, 0), 0)
        tc_parm = vsp.GetXSecParm(xsec, "ThickChord")
        vsp.SetParmVal(tc_parm, 0.12)

        # Save file
        fname = "dbf.vsp3"

        vsp.SetVSP3FileName( fname )

        vsp.Update()

        #==== Save Vehicle to File ====//
        print( "\tSaving vehicle file to: ", fname )
        vsp.WriteVSPFile( vsp.GetVSPFileName(), SET_ALL )
        vsp.ExportFile( "dbf.stl", SET_ALL, EXPORT_STL)

        mesh_id = vsp.ComputeCompGeom(vsp.SET_ALL, False, COMP_GEOM_TXT_TYPE)
        comp_geom_res_id = vsp.FindLatestResultsID("Comp_Geom")
        areas = vsp.GetDoubleResults(comp_geom_res_id, "Wet_Area")

        fuse_wet_area = areas[0]
        wing_wet_area = areas[1]
        htail_wet_area = areas[2]
        vtail_wet_area = areas[3]

        outputs[Aircraft.Fuselage.WETTED_AREA] = fuse_wet_area
        outputs[Aircraft.Wing.WETTED_AREA] = wing_wet_area
        outputs[Aircraft.HorizontalTail.WETTED_AREA] = htail_wet_area
        outputs[Aircraft.VerticalTail.WETTED_AREA] = vtail_wet_area

        # print(f"Fuselage Wetted Area: {fuse_wet_area:.3f} inch**2")
        # print(f"Wing Wetted Area: {wing_wet_area:.3f} inch**2")
        # print(f"HTail Wetted Area: {htail_wet_area:.3f} inch**2")
        # print(f"VTail Wetted Area: {vtail_wet_area:.3f} inch**2")

        #==== Reset Geometry ====//
        print( "--->Resetting VSP model to blank slate\n" )

        vsp.ClearVSPModel()

    

if __name__ == "__main__":
    prob = om.Problem()

    prob.model.add_subsystem(
        'dbf_wing', DBFGeom(), promotes_inputs=['*'], promotes_outputs=['*']
    )

    prob.setup()

    # Set values for mesh-driving variables
    prob.set_val(Aircraft.Wing.ROOT_CHORD, val=20, units='inch')
    prob.set_val(Aircraft.Wing.SPAN, val=4.667, units='ft')
    prob.set_val(Aircraft.HorizontalTail.ROOT_CHORD, val=8.75, units='inch')
    prob.set_val(Aircraft.HorizontalTail.SPAN, val=28.0, units='inch')
    prob.set_val(Aircraft.VerticalTail.ROOT_CHORD, val=8.75, units='inch')
    prob.set_val(Aircraft.VerticalTail.SPAN, val=1, units='ft')
    prob.set_val(Aircraft.Fuselage.LENGTH, val=6, units='ft')
    prob.set_val(Aircraft.Fuselage.AVG_WIDTH, val=5, units='inch')
    prob.set_val(Aircraft.Fuselage.AVG_HEIGHT, val=4, units='inch')


    prob.run_model()