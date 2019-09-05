
import pathlib

from lsst.ts import salobj

# Aliases
state = salobj.base_csc.State

class Hexapod(salobj.ConfigurableCsc):
    """docstring for Hexapod"""

    def __init__(self, config_dir=None, initial_state=state.STANDBY,
                 initial_simulation_mode=0):
        """
        Initialize Hexapod CSC.

        Parameters
        -----------
        config_dir : `str` (optional)
            Directory of configuration files, or None for the standard
            configuration directory (obtained from `get_default_config_dir`).
            This is provided for unit testing.
        initial_state : `salobj.State` (optional)
            The initial state of the CSC. Typically one of:
            - State.ENABLED if you want the CSC immediately usable.
            - State.STANDBY if you want full emulation of a CSC.
        initial_simulation_mode : `int` (optional)
            Initial simulation mode. This is provided for unit testing,
            as real CSCs should start up not simulating, the default.
        index : int
            Index for the DIMM. This enables the control of multiple DIMMs.
        """
        schema_path = \
            pathlib.Path(__file__).resolve().parents[4].joinpath("schema", "Hexapod.yaml")

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
        Configuration, as described by ``schema/Hexapod.yaml``
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
