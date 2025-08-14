import unittest

import aviary.api as av

DBFGeometryBuilder = av.TestSubsystemBuilderBase.import_builder(
    'dbf_based_geometry.dbf_geometry_builder.DBFGeometryBuilder'
)


@av.skipIfMissingDependencies(DBFGeometryBuilder)
class TestDBFMass(av.TestSubsystemBuilderBase):
    """Test geom builder."""

    def setUp(self):
        self.subsystem_builder = DBFGeometryBuilder()
        self.aviary_values = av.AviaryValues()


if __name__ == '__main__':
    unittest.main()
