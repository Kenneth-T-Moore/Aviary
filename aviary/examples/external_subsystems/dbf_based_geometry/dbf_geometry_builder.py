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
        return GeomPremission()

    def get_design_vars(self):
        """
        Design vars are only tested to see if they exist in pre_mission
        Returns a dictionary of design variables for the gearbox subsystem, where the keys are the
        names of the design variables, and the values are dictionaries that contain the units for
        the design variable, the lower and upper bounds for the design variable, and any
        additional keyword arguments required by OpenMDAO for the design variable.

        Returns
        -------
        parameters : dict
        A dict of names for the propeller subsystem.
        """
        # TODO Nish bounds are rough placeholders
        # TODO Nish once OAS is in add CG DVs?
        DVs = {
            Aircraft.Wing.ROOT_CHORD: {
                'units': 'inch',
                'lower': 3,
                'upper': 36,
                # 'val': 100,  
            },

            Aircraft.Wing.SPAN: {
                'units': 'ft',
                'lower': 2,
                'upper': 10,
                # 'val': 100,  
            },

            Aircraft.Wing.CENTER_DISTANCE: {
                'units': 'unitless',
                'lower': 0,
                'upper': 1,
                # 'val': 100,  
            },
            
            Aircraft.HorizontalTail.ROOT_CHORD: {
                'units': 'inch',
                'lower': 3,
                'upper': 36,
                # 'val': 100,  
            },

            Aircraft.HorizontalTail.SPAN: {
                'units': 'ft',
                'lower': 1,
                'upper': 5,
                # 'val': 100,  
            },

            Aircraft.VerticalTail.ROOT_CHORD: {
                'units': 'inch',
                'lower': 3,
                'upper': 24,
                # 'val': 100,  
            },

            Aircraft.VerticalTail.SPAN: {
                'units': 'ft',
                'lower': 0.5,
                'upper': 3,
                # 'val': 100,  
            },

            Aircraft.Fuselage.LENGTH: {
                'units': 'ft',
                'lower': 1,
                'upper': 7,
                # 'val': 100,  
            },

            Aircraft.Fuselage.AVG_WIDTH: {
                'units': 'm',
                'lower': 0.01,
                'upper': 0.5,
                # 'val': 100,  
            },

            Aircraft.Fuselage.AVG_HEIGHT: {
                'units': 'm',
                'lower': 0.01,
                'upper': 0.5,
                # 'val': 100,  
            },

        }
        return DVs
