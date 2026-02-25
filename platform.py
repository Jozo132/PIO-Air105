"""
PIO-Air105 — PlatformIO Platform Class
=======================================
Custom platform for Air105 (MH1903S) Cortex-M4F MCU.
Supports the Arduino framework.

@author J.Vovk <Jozo132@gmail.com>
@url    https://github.com/Jozo132/PIO-Air105
SPDX-License-Identifier: MIT
"""

import os
from platformio.public import PlatformBase


class Air105Platform(PlatformBase):

    def get_dir(self):
        """Always return an absolute path so SCons can resolve SConscript paths."""
        return os.path.abspath(super().get_dir())
