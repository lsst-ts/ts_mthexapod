
import pathlib

from lsst.ts import salobj


class Hexapod(salobj.ConfigurableCsc):
    """docstring for Hexapod"""

    def __init__(self, config_dir=None, initial_state=salobj.base_csc.State.STANDBY,
                 initial_simulation_mode=0):
        schema_path = pathlib.Path(__file__).resolve().parents[4].joinpath("schema", "Hexapod.yaml")

        super().__init__(name="Hexapod", schema_path=schema_path, config_dir=config_dir,
                         index=None, initial_state=initial_state,
                         initial_simulation_mode=initial_simulation_mode)

    @staticmethod
    def get_config_pkg():
        return "ts_config_mttcs"

    async def configure(self, config):
        """Configure this CSC and output the ``settingsApplied`` event.

        Parameters
        ----------
        config : `types.SimpleNamespace`
        Configuration, as described by ``schema/Environment.yaml``
        """
        self.config = config

    async def do_configureAcceleration(self, id_data):
        """ """
        pass

    async def do_configureLimits(self, id_data):
        """ """
        pass

    async def do_configureElevationRawLUT(self, id_data):
        """ """
        pass

    async def do_move(self, id_data):
        """ """
        pass

    async def do_positionSet(self, id_data):
        """ """
        pass

    async def do_configureVelocity(self, id_data):
        """ """
        pass

    async def do_offset(self, id_data):
        """ """
        pass

    async def do_pivot(self, id_data):
        """ """
        pass

    async def do_clearError(self, id_data):
        """ """
        pass

    async def do_test(self, id_data):
        """ """
        pass

    async def do_configureAzimuthRawLUT(self, id_data):
        """ """
        pass

    async def do_configureTemperatureRawLUT(self, id_data):
        """ """
        pass

    async def do_moveLUT(self, id_data):
        """ """
        pass
