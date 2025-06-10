"""
Builder for an external subsystem that contains an Aviary submodel performing detailed landing.
"""
from aviary.examples.external_subsystems.detailed_landing.landing_submodel import (
    ExternalLandingGroup,
)
from aviary.subsystems.subsystem_builder_base import SubsystemBuilderBase


class DetailedLandingBuilder(SubsystemBuilderBase):
    """
    Builder for a subsystem that contains an Aviary submodel performing detailed landing.
    """
    def __init__(self, name='detailed_landing'):
        super().__init__(name)

    def build_post_mission(self, aviary_inputs, **kwargs):
        return ExternalLandingGroup(aviary_inputs=aviary_inputs)