import openvsp as vsp 
import os
import openmdao.api as om
from openvsp import SET_ALL, EXPORT_STL

class DBFAircraftMeshExporter(om.ExplicitComponent):

    def setup(self):
        # Inputs (Design Variables)
        self.add_input("wing_span", val=4.0, units="m", desc="Wing span")
        self.add_input("wing_chord", val=0.5, units="m", desc="Wing chord")

        # Outputs
        self.add_output("initial_mesh_path", val="", desc="Path to initial mesh file", units=None)
        self.add_output("final_mesh_path", val="", desc="Path to final mesh file", units=None)

        # Internals
        self.model_path = os.path.abspath("test.vsp3")
        self.initial_mesh_out = os.path.abspath("initial_mesh.stl")
        self.final_mesh_out = os.path.abspath("final_mesh.stl")

    def premission(self):
        self._generate_mesh(self.model_path, self.initial_mesh_out)

    def postmission(self):
        self._generate_mesh(self.model_path, self.final_mesh_out)

    def compute(self, inputs, outputs):

        model_path = self.model_path

        status = vsp.ReadVSPFile(model_path)
        if status != 0:
            raise RuntimeError(f"Failed to read VSP file: {model_path} (error code {status})")
        # Store output paths
        outputs["initial_mesh_path"] = self.initial_mesh_out
        outputs["final_mesh_path"] = self.final_mesh_out

    def _generate_mesh(self, model_path, output_path):
        # Reset and load model
        vsp.VSPRenew()
        vsp.ReadVSPFile(model_path)

        # Find and update parameters
        wing_id = vsp.FindGeom("Wing", 0)
        span_id = vsp.GetParm(wing_id, "Span", "XSec_1")
        chord_id = vsp.GetParm(wing_id, "Chord", "XSec_1")

        vsp.SetParmVal(span_id, float(self.get_val("wing_span")))
        vsp.SetParmVal(chord_id, float(self.get_val("wing_chord")))
        vsp.Update()

        # Run CompGeom to generate mesh
        compgeom_id = vsp.ExecAnalysis("CompGeom")
        mesh_ids = vsp.GetStringResults(compgeom_id, "Mesh_GeomID")

        # Export mesh to STL
        vsp.ExportFile(output_path, SET_ALL, EXPORT_STL)

if __name__ == "__main__":
    import shutil
    import pathlib
    
    class MeshGroup(om.Group):
        def setup(self):
            self.add_subsystem("mesh_exporter", DBFAircraftMeshExporter(), promotes=["*"])

    # Set up OpenMDAO problem
    prob = om.Problem()
    prob.model = MeshGroup()  # Use a Group wrapper

    prob.setup()

    # Debug print: available geoms and parms
    print(vsp.FindGeom("HELLY", 0))

    for geom_id in vsp.FindGeomsWithName("Wing: Wing"):
        print(f"Geom: {geom_id}")

    wing_id = vsp.FindGeom("Wing: Helly", 0)  # Or use the correct name from VSP
    if not wing_id:
        raise RuntimeError("Wing not found!")

    parms = vsp.GetParmVal(wing_id)
    for pid in parms:
        print(f"Parm: {vsp.GetParmName(pid)} | Group: {vsp.GetParmGroupName(pid)}")

    # Set some example design variable values
    prob.set_val("wing_span", 5.0, units="m")
    prob.set_val("wing_chord", 0.75, units="m")

    # Run premission (initial geometry/mesh export)
    print("Generating initial mesh...")
    prob.model.mesh_exporter.premission()

    # Run compute (records outputs)
    prob.run_model()

    # Simulate design update and postmission
    print("Generating final mesh...")
    prob.model.mesh_exporter.postmission()

    # Print mesh file paths
    initial_mesh = prob.get_val("initial_mesh_path")
    final_mesh = prob.get_val("final_mesh_path")

    print(f"Initial mesh exported to: {initial_mesh}")
    print(f"Final mesh exported to:   {final_mesh}")

    # Optional: check that files exist
    for path in [initial_mesh, final_mesh]:
        if pathlib.Path(path).exists():
            print(f"Mesh file exists: {path}")
        else:
            print(f"Mesh file not found: {path}")
