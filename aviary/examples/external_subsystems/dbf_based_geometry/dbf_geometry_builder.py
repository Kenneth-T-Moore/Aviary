import openmdao.api as om

import aviary as av
from aviary.variable_info.variables import Aircraft
from aviary.subsystems.subsystem_builder_base import SubsystemBuilderBase
from aviary.examples.external_subsystems.dbf_based_geometry.dbf_geometry_premission import GeomPremission


class DBFGeometryBuilder(SubsystemBuilderBase):
    """
    Builder for DBF mass models including wing, horizontal tail, vertical tail, and fuselage.
    """

    def __init__(self, name='dbf_geom'):
        if name is None:
            name = _default_name

        super().__init__(name=name)

    def build_pre_mission(self, aviary_inputs):
        return MassPremission()

