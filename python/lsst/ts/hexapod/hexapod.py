
import pathlibs

from lsst.ts import salobj


class Hexapod(salobj.ConfigurableCsc):
	"""docstring for Hexapod"""

	def __init__(self, config_dir=None, initial_state=salobj.base_csc.State.STANDBY,
                 initial_simulation_mode=0):

        schema_path = pathlib.Path(__file__).resolve().parents[4].join("schema", "Hexapod.yaml")

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


	async def do_configureAcceleration
		""" """
		pass

		

    async def configureLimits(self, id_data):
		""" """
		pass



    async def configureElevationRawLUT(self, id_data):
		""" """
		pass


    async def move(self, id_data):
		""" """
		pass


    async def positionSet(self, id_data):
		""" """
		pass


    async def configureVelocity(self, id_data):
		""" """
		pass


    async def offset(self, id_data):
		""" """
		pass


    async def pivot(self, id_data):
		""" """
		pass


    async def clearError(self, id_data):
		""" """
		pass


    async def test(self, id_data):
		""" """
		pass


    async def configureAzimuthRawLUT(self, id_data):
		""" """
		pass


    async def configureTemperatureRawLUT(self, id_data):
		""" """
		pass


    async def moveLUT(self, id_data):
    	""" """
		pass


