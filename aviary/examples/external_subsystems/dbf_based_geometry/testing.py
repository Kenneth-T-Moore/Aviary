import openvsp as vsp
from openvsp import (SET_ALL, XS_ROUNDED_RECTANGLE, XS_FOUR_SERIES, EXPORT_STL,
                     FLAT_END_CAP, EDGE_END_CAP, NO_END_CAP, ROUND_END_CAP)

# Add Fuselage Geom
fuseid = vsp.AddGeom("FUSELAGE", "")
vsp.SetParmVal(fuseid, "Length", "Design", 30.0)  # Set desired fuselage length

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
            vsp.SetParmVal(zid, 0.033)
            wid = vsp.GetXSecParm(xsec, "RoundedRect_Width")
            vsp.SetParmVal(wid, 0.5)
            hid = vsp.GetXSecParm(xsec, "RoundedRect_Height")
            vsp.SetParmVal(hid, 0.5)
            taid = vsp.GetXSecParm(xsec, "TopLAngle")
            vsp.SetParmVal(taid, 10.0)
            raid = vsp.GetXSecParm(xsec, "RightLAngle")
            vsp.SetParmVal(raid, 10.0)
            tsid = vsp.GetXSecParm(xsec, "TopLStrength")
            vsp.SetParmVal(tsid, 0.0)
            rsid = vsp.GetXSecParm(xsec, "RightLStrength")
            vsp.SetParmVal(rsid, 0.0)
        
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
            vsp.SetParmVal(wid, 5.0)
            hid = vsp.GetXSecParm(xsec, "RoundedRect_Height")
            vsp.SetParmVal(hid, 4.0)

    fcap = vsp.GetParm(fuseid, "CapUMinOption", "EndCap")
    vsp.SetParmVal(fcap, ROUND_END_CAP)
    fcap_length = vsp.GetParm(fuseid, "CapUMinLength", "EndCap")
    vsp.SetParmVal(fcap_length, 0.015 * 30.0)
    ecap = vsp.GetParm(fuseid, "CapUMaxOption", "EndCap")
    vsp.SetParmVal(ecap, FLAT_END_CAP)


vsp.Update()

### Wing ###
wid = vsp.AddGeom( "WING", "" ) # Add Wing
# Set wing parameters to remove taper and sweep
vsp.SetParmVal(wid, "Root_Chord", "XSec_1", 6.0)   # Root chord length
vsp.SetParmVal(wid, "Tip_Chord", "XSec_1", 6.0)    # Tip chord = Root chord → no taper
vsp.SetParmVal(wid, "Sweep", "XSec_1", 0.0)        # No sweep
vsp.SetParmVal(wid, "Dihedral", "XSec_1", 0.0)     # (optional) No dihedral
vsp.SetParmVal(wid, "Span", "XSec_1", 12.0)        # Set desired span
vsp.SetParmVal(wid, "Twist", "XSec_1", 0.0)        # (optional) No twist
vsp.SetParmVal(wid, "X_Rel_Location", "XForm", 9.0)
vsp.SetParmVal(wid, "Z_Rel_Location", "XForm", -0.5)

# After creating the wing and setting span, chord, etc.
xsec = vsp.GetXSec(vsp.GetXSecSurf(wid, 0), 1)     # Get the actual XSec object
tc_parm = vsp.GetXSecParm(xsec, "ThickChord")      # Get the t/c parameter
vsp.SetParmVal(tc_parm, 0.2)                       # Set thickness-to-chord ratio (e.g., 15%)
xsec = vsp.GetXSec(vsp.GetXSecSurf(wid, 0), 0)     # Get the actual XSec object
tc_parm = vsp.GetXSecParm(xsec, "ThickChord")      # Get the t/c parameter
vsp.SetParmVal(tc_parm, 0.2)                       # Set thickness-to-chord ratio (e.g., 15%)

### HTAIL ###
htail_id = vsp.AddGeom( "WING", "" ) # Add Wing
# Set wing parameters to remove taper and sweep
vsp.SetParmVal(htail_id, "Root_Chord", "XSec_1", 3.0)   # Root chord length
vsp.SetParmVal(htail_id, "Tip_Chord", "XSec_1", 3.0)    # Tip chord = Root chord → no taper
vsp.SetParmVal(htail_id, "Sweep", "XSec_1", 0.0)        # No sweep
vsp.SetParmVal(htail_id, "Dihedral", "XSec_1", 0.0)     # (optional) No dihedral
vsp.SetParmVal(htail_id, "Span", "XSec_1", 6.0)         # Set desired span
vsp.SetParmVal(htail_id, "Twist", "XSec_1", 0.0)        # (optional) No twist
vsp.SetParmVal(htail_id, "X_Rel_Location", "XForm", 30.0)
vsp.SetParmVal(htail_id, "Z_Rel_Location", "XForm", 1.0)

# After creating the wing and setting span, chord, etc.
xsec = vsp.GetXSec(vsp.GetXSecSurf(htail_id, 0), 1)     # Get the actual XSec object
tc_parm = vsp.GetXSecParm(xsec, "ThickChord")      # Get the t/c parameter
vsp.SetParmVal(tc_parm, 0.15)                      # Set thickness-to-chord ratio (e.g., 15%)
xsec = vsp.GetXSec(vsp.GetXSecSurf(wid, 0), 0)     # Get the actual XSec object
tc_parm = vsp.GetXSecParm(xsec, "ThickChord")      # Get the t/c parameter
vsp.SetParmVal(tc_parm, 0.15)                      # Set thickness-to-chord ratio (e.g., 15%)

### VTAIL ###
vtail_id = vsp.AddGeom("WING", "")

# Set vertical tail parameters
vsp.SetParmVal(vtail_id, "Root_Chord", "XSec_1", 3.0)
vsp.SetParmVal(vtail_id, "Tip_Chord", "XSec_1", 3.0)
vsp.SetParmVal(vtail_id, "Span", "XSec_1", 5.0)            # Acts as "height" here
vsp.SetParmVal(vtail_id, "Sweep", "XSec_1", 0.0)
vsp.SetParmVal(vtail_id, "Twist", "XSec_1", 0.0)

# Move it to the rear of the fuselage
vsp.SetParmVal(vtail_id, "X_Rel_Location", "XForm", 30.0)
vsp.SetParmVal(vtail_id, "Z_Rel_Location", "XForm", 1.0)

# Rotate to make it vertical (pitch = 90 degrees)
vsp.SetParmVal(vtail_id, "X_Rel_Rotation", "XForm", 90.0)

# Set airfoil shape and thickness/chord ratio
xsec = vsp.GetXSec(vsp.GetXSecSurf(vtail_id, 0), 1)
tc_parm = vsp.GetXSecParm(xsec, "ThickChord")
vsp.SetParmVal(tc_parm, 0.15)
xsec = vsp.GetXSec(vsp.GetXSecSurf(vtail_id, 0), 0)
tc_parm = vsp.GetXSecParm(xsec, "ThickChord")
vsp.SetParmVal(tc_parm, 0.15)

# Save file
fname = "dbf.vsp3"

vsp.SetVSP3FileName( fname )

vsp.Update()

#==== Save Vehicle to File ====//
print( "\tSaving vehicle file to: ", fname )
vsp.WriteVSPFile( vsp.GetVSPFileName(), SET_ALL )
vsp.ExportFile( "dbf.stl", SET_ALL, EXPORT_STL)

#==== Reset Geometry ====//
print( "--->Resetting VSP model to blank slate\n" )

vsp.ClearVSPModel()