import openvsp as vsp
from openvsp import SET_ALL
import numpy as np
import os

def load_airfoil_csv(file_path, delimiter=',', header=False):
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

upper_pnts, lower_pnts = load_airfoil_csv("aviary/examples/external_subsystems/dbf_based_mass/mh84-il.csv", 
                                  delimiter=",", 
                                  header=True)



# === Build the geometry ===
wing_id = vsp.AddGeom("WING")
xsec_surf_id = vsp.GetXSecSurf(wing_id, 0)

# Loop over all cross sections
for xsec_index in range(vsp.GetNumXSec(xsec_surf_id)):
    vsp.ChangeXSecShape(xsec_surf_id, xsec_index, vsp.XS_FILE_AIRFOIL)
    xsec_id = vsp.GetXSec(xsec_surf_id, xsec_index)
    vsp.SetAirfoilPnts(xsec_id, upper_pnts, lower_pnts)


# === Update and generate geometry ===
vsp.Update()

fname = "test.vsp3"

vsp.SetVSP3FileName( fname )

#==== Save Vehicle to File ====//
print( "\tSaving vehicle file to: ", fname )
vsp.WriteVSPFile( vsp.GetVSPFileName(), SET_ALL )

vsp.Update()

vsp.ClearVSPModel()