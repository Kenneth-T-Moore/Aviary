import openvsp as vsp
from openvsp import SET_ALL, XS_ROUNDED_RECTANGLE, XS_FOUR_SERIES


# Add Fuselage Geom
fuseid = vsp.AddGeom("FUSELAGE", "")

# Loop over all fuselage sections
num_sections = vsp.GetNumXSecSurfs(fuseid)
for i in range(num_sections):

    xsec_surf = vsp.GetXSecSurf(fuseid, i)
    num_xsecs = vsp.GetNumXSec(xsec_surf)

    for j in range(num_xsecs):

        if j == 0 or j == (num_xsecs - 1):
            vsp.ChangeXSecShape(xsec_surf, j, XS_ROUNDED_RECTANGLE)
            xsec = vsp.GetXSec(xsec_surf, j)
            wid = vsp.GetXSecParm(xsec, "RoundedRect_Width")
            vsp.SetParmVal(wid, 0.0)
            lid = vsp.GetXSecParm(xsec, "RoundedRect_Length")
            vsp.SetParmVal(lid, 3.0)

        else:
            vsp.ChangeXSecShape(xsec_surf, j, XS_ROUNDED_RECTANGLE)
            xsec = vsp.GetXSec(xsec_surf, j)
            wid = vsp.GetXSecParm(xsec, "RoundedRect_Width")
            vsp.SetParmVal(wid, 4.0)

fname = "example_fuse.vsp3"

vsp.SetVSP3FileName( fname )

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
vsp.ChangeXSecShape(wid, 1, XS_FOUR_SERIES)        # Set the wing's first section to FOIL type
xsec = vsp.GetXSec(vsp.GetXSecSurf(wid, 0), 1)     # Get the actual XSec object
tc_parm = vsp.GetXSecParm(xsec, "ThickChord")      # Get the t/c parameter
vsp.SetParmVal(tc_parm, 0.2)                       # Set thickness-to-chord ratio (e.g., 15%)
vsp.ChangeXSecShape(wid, 0, XS_FOUR_SERIES)        # Set the wing's first section to FOIL type
xsec = vsp.GetXSec(vsp.GetXSecSurf(wid, 0), 0)     # Get the actual XSec object
tc_parm = vsp.GetXSecParm(xsec, "ThickChord")      # Get the t/c parameter
vsp.SetParmVal(tc_parm, 0.2)                       # Set thickness-to-chord ratio (e.g., 15%)

### HTAIL ###
wid = vsp.AddGeom( "WING", "" ) # Add Wing
# Set wing parameters to remove taper and sweep
vsp.SetParmVal(wid, "Root_Chord", "XSec_1", 3.0)   # Root chord length
vsp.SetParmVal(wid, "Tip_Chord", "XSec_1", 3.0)    # Tip chord = Root chord → no taper
vsp.SetParmVal(wid, "Sweep", "XSec_1", 0.0)        # No sweep
vsp.SetParmVal(wid, "Dihedral", "XSec_1", 0.0)     # (optional) No dihedral
vsp.SetParmVal(wid, "Span", "XSec_1", 6.0)         # Set desired span
vsp.SetParmVal(wid, "Twist", "XSec_1", 0.0)        # (optional) No twist
vsp.SetParmVal(wid, "X_Rel_Location", "XForm", 27.0)

# After creating the wing and setting span, chord, etc.
vsp.ChangeXSecShape(wid, 1, XS_FOUR_SERIES)        # Set the wing's first section to FOIL type
xsec = vsp.GetXSec(vsp.GetXSecSurf(wid, 0), 1)     # Get the actual XSec object
tc_parm = vsp.GetXSecParm(xsec, "ThickChord")      # Get the t/c parameter
vsp.SetParmVal(tc_parm, 0.15)                      # Set thickness-to-chord ratio (e.g., 15%)
vsp.ChangeXSecShape(wid, 0, XS_FOUR_SERIES)        # Set the wing's first section to FOIL type
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
vsp.SetParmVal(vtail_id, "X_Rel_Location", "XForm", 27.0)
vsp.SetParmVal(vtail_id, "Z_Rel_Location", "XForm", 0.0)

# Rotate to make it vertical (pitch = 90 degrees)
vsp.SetParmVal(vtail_id, "X_Rel_Rotation", "XForm", 90.0)

# Set airfoil shape and thickness/chord ratio
vsp.ChangeXSecShape(vtail_id, 1, XS_FOUR_SERIES)
xsec = vsp.GetXSec(vsp.GetXSecSurf(vtail_id, 0), 1)
tc_parm = vsp.GetXSecParm(xsec, "ThickChord")
vsp.SetParmVal(tc_parm, 0.15)

vsp.ChangeXSecShape(vtail_id, 0, XS_FOUR_SERIES)
xsec = vsp.GetXSec(vsp.GetXSecSurf(vtail_id, 0), 0)
tc_parm = vsp.GetXSecParm(xsec, "ThickChord")
vsp.SetParmVal(tc_parm, 0.15)


fname = "dbf.vsp3"

vsp.SetVSP3FileName( fname )

vsp.Update()

#==== Save Vehicle to File ====//
print( "\tSaving vehicle file to: ", False )

print( fname )

vsp.WriteVSPFile( vsp.GetVSPFileName(), SET_ALL )

#==== Reset Geometry ====//
print( "--->Resetting VSP model to blank slate\n" )

vsp.ClearVSPModel()