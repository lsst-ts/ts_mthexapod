import logging
import pathlib
import asyncio
import numpy as np
import unittest

from lsst.ts import salobj
from lsst.ts.hexapod import Hexapod

np.random.seed(50)
index_gen = salobj.index_generator()

logger = logging.getLogger()
logger.level = logging.DEBUG

STD_TIMEOUT = 2  # standard command timeout (sec)
LONG_TIMEOUT = 20  # time limit for starting a SAL component (sec)
TEST_CONFIG_DIR = pathlib.Path(__file__).resolve().parents[1].joinpath("tests", "data", "config")


class Harness:
    def __init__(self, initial_state=salobj.base_csc.State.STANDBY,
                 config_dir=None, initial_simulation_mode=0):
        salobj.test_utils.set_random_lsst_dds_domain()

        self.csc = Hexapod(initial_state=initial_state,
                           config_dir=config_dir,
                           initial_simulation_mode=initial_simulation_mode)
        self.remote = salobj.Remote(domain=self.csc.domain, name="Hexapod")

    async def __aenter__(self):
        await asyncio.gather(self.csc.start_task,
                             self.remote.start_task)
        return self

    async def __aexit__(self, *args):
        await self.csc.close()


class HexapodTestCase(unittest.TestCase):

    def test_standard_state_transitions(self):
        """Test standard CSC state transitions.

        The initial state is STANDBY.
        The standard commands and associated state transitions are:

        * enterControl: OFFLINE to STANDBY
        * start: STANDBY to DISABLED
        * enable: DISABLED to ENABLED

        * disable: ENABLED to DISABLED
        * standby: DISABLED to STANDBY
        * exitControl: STANDBY, FAULT to OFFLINE (quit)
        """

        async def doit():

            async with Harness(config_dir=TEST_CONFIG_DIR) as harness:

                evt_timeout = 5

                state = await harness.remote.evt_summaryState.next(flush=False, timeout=evt_timeout)
                self.assertEqual(salobj.State(state.summaryState), salobj.State.STANDBY)

        asyncio.get_event_loop().run_until_complete(doit())


if __name__ == "__main__":
    unittest.main()
