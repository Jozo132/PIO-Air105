"""
PIO-Air105 — Arduino Framework Builder
=======================================
Configures include paths and compiles framework source files
(cores, variants, system) for the Arduino-compatible framework.

Called from builder/main.py when ``framework = arduino`` is set.

@author J.Vovk <Jozo132@gmail.com>
@url    https://github.com/Jozo132/PIO-Air105
SPDX-License-Identifier: MIT
"""

import os
import glob
from os.path import join, basename

from SCons.Script import DefaultEnvironment, Import

env = DefaultEnvironment()
Import("env")

platform = env.PioPlatform()
board = env.BoardConfig()

# Platform root directory (where platform.json lives)
PLATFORM_DIR = platform.get_dir()

# Resolve core and variant from board manifest
CORE_NAME = board.get("build.core", "air105")
VARIANT_NAME = board.get("build.variant", "air105_devboard")

CORES_DIR = join(PLATFORM_DIR, "cores", CORE_NAME)
VARIANTS_DIR = join(PLATFORM_DIR, "variants", VARIANT_NAME)
SYSTEM_DIR = join(PLATFORM_DIR, "system")
CMSIS_DIR = join(SYSTEM_DIR, "cmsis")

# ============================================================================
# Validate paths
# ============================================================================

for label, path in [
    ("Cores", CORES_DIR),
    ("Variants", VARIANTS_DIR),
    ("System", SYSTEM_DIR),
    ("CMSIS", CMSIS_DIR),
]:
    if not os.path.isdir(path):
        env.Exit("Error: %s directory not found: %s" % (label, path))

# ============================================================================
# Banner
# ============================================================================

print("=" * 60)
print("PIO-Air105 Arduino Framework")
print("=" * 60)
print("  Platform: %s" % PLATFORM_DIR)
print("  Core:     %s" % CORES_DIR)
print("  Variant:  %s" % VARIANTS_DIR)
print("  System:   %s" % SYSTEM_DIR)
print("  CMSIS:    %s" % CMSIS_DIR)
print("=" * 60)

# ============================================================================
# Include paths
# ============================================================================

env.Append(
    CPPPATH=[
        CORES_DIR,
        VARIANTS_DIR,
        SYSTEM_DIR,
        CMSIS_DIR,
    ]
)

# ============================================================================
# Linker script
# ============================================================================

ldscript = board.get("build.ldscript", "air105_flash.ld")
ldscript_path = join(SYSTEM_DIR, ldscript)

if not os.path.isfile(ldscript_path):
    env.Exit("Error: Linker script not found: %s" % ldscript_path)

env.Append(LINKFLAGS=["-T", ldscript_path])

# ============================================================================
# Framework source files
# ============================================================================

# Core files (Arduino API + system startup/init)
env.BuildSources(
    join("$BUILD_DIR", "FrameworkArduino"),
    CORES_DIR,
    src_filter=["+<*.c>", "+<*.cpp>", "+<*.S>"],
)
core_sources = (
    glob.glob(join(CORES_DIR, "*.c"))
    + glob.glob(join(CORES_DIR, "*.cpp"))
    + glob.glob(join(CORES_DIR, "*.S"))
)
for src in sorted(core_sources):
    print("  [Core] %s" % basename(src))

# Variant files (board-specific pin maps)
env.BuildSources(
    join("$BUILD_DIR", "FrameworkVariant"),
    VARIANTS_DIR,
    src_filter=["+<*.c>", "+<*.cpp>"],
)
variant_sources = glob.glob(join(VARIANTS_DIR, "*.c")) + glob.glob(
    join(VARIANTS_DIR, "*.cpp")
)
for src in sorted(variant_sources):
    print("  [Variant] %s" % basename(src))

print("=" * 60)
