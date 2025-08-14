import openmdao.api as om

from aviary.examples.external_subsystems.dbf_based_geometry.dbf_geometry import DBFGeom

from aviary.variable_info.variables import Aircraft


class GeomPremission(om.Group):
    def setup(self):
        self.add_subsystem(
            'dbf_geom', 
            DBFGeom(), 
            promotes_inputs=['*'], 
            promotes_outputs=[
                Aircraft.Fuselage.WETTED_AREA,
                Aircraft.Wing.WETTED_AREA,
                Aircraft.HorizontalTail.WETTED_AREA,
                Aircraft.VerticalTail.WETTED_AREA,
                ],
        )
