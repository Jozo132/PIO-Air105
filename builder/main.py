"""
PIO-Air105 — Main Builder Script
=================================
Configures the ARM GCC toolchain, build flags, build targets,
and upload method for the Air105 (MH1903S) MCU.

Called by PlatformIO as the platform's build entry point.

@author J.Vovk <Jozo132@gmail.com>
@url    https://github.com/Jozo132/PIO-Air105
SPDX-License-Identifier: MIT
"""

import sys
import os
from os.path import join, isfile

from SCons.Script import (
    COMMAND_LINE_TARGETS,
    AlwaysBuild,
    Builder,
    Default,
    DefaultEnvironment,
)

env = DefaultEnvironment()
platform = env.PioPlatform()
board = env.BoardConfig()

# Platform root directory (where platform.json lives)
PLATFORM_DIR = platform.get_dir()

# ============================================================================
# Toolchain executables
# ============================================================================

env.Replace(
    AR="arm-none-eabi-ar",
    AS="arm-none-eabi-as",
    CC="arm-none-eabi-gcc",
    CXX="arm-none-eabi-g++",
    GDB="arm-none-eabi-gdb",
    OBJCOPY="arm-none-eabi-objcopy",
    RANLIB="arm-none-eabi-ranlib",
    SIZETOOL="arm-none-eabi-size",

    ARFLAGS=["rc"],

    SIZEPROGREGEXP=r"^(?:\.text|\.data|\.rodata|\.text\.align|\.ARM\.exidx)\s+(\d+).*",
    SIZEDATAREGEXP=r"^(?:\.data|\.bss|\.noinit)\s+(\d+).*",
    SIZECHECKCMD="$SIZETOOL -A -d $SOURCES",
    SIZEPRINTCMD="$SIZETOOL -B -d $SOURCES",

    PROGSUFFIX=".elf",
)

# ============================================================================
# Build flags (Cortex-M4F with hardware FPU)
# ============================================================================

env.Append(
    CCFLAGS=[
        "-mcpu=cortex-m4",
        "-mthumb",
        "-mfloat-abi=hard",
        "-mfpu=fpv4-sp-d16",
        "-Os",
        "-Wall",
        "-ffunction-sections",
        "-fdata-sections",
        "-fno-exceptions",
    ],
    CXXFLAGS=[
        "-fno-rtti",
        "-std=gnu++17",
    ],
    CFLAGS=[
        "-std=gnu11",
    ],
    ASFLAGS=[
        "-mcpu=cortex-m4",
        "-mthumb",
        "-mfloat-abi=hard",
        "-mfpu=fpv4-sp-d16",
    ],
    LINKFLAGS=[
        "-mcpu=cortex-m4",
        "-mthumb",
        "-mfloat-abi=hard",
        "-mfpu=fpv4-sp-d16",
        "-Wl,--gc-sections",
        "-Wl,--check-sections",
        "--specs=nano.specs",
        "--specs=nosys.specs",
    ],
    CPPDEFINES=[
        "AIR105",
        "ARM_MATH_CM4",
        ("__FPU_PRESENT", "1"),
    ],
    LIBS=["m", "c", "gcc", "stdc++", "nosys"],
)

# Board-specific extra flags (e.g. -DAIR105 from board manifest)
if board.get("build.extra_flags", ""):
    env.Append(CCFLAGS=board.get("build.extra_flags", "").split())

# ============================================================================
# Build targets: ELF → BIN
# ============================================================================

env.Append(
    BUILDERS=dict(
        ElfToBin=Builder(
            action=env.VerboseAction(
                " ".join([
                    "$OBJCOPY",
                    "-O", "binary",
                    "$SOURCES",
                    "$TARGET",
                ]),
                "Building $TARGET",
            ),
            suffix=".bin",
        ),
        ElfToHex=Builder(
            action=env.VerboseAction(
                " ".join([
                    "$OBJCOPY",
                    "-O", "ihex",
                    "$SOURCES",
                    "$TARGET",
                ]),
                "Building $TARGET",
            ),
            suffix=".hex",
        ),
    )
)

if "nobuild" in COMMAND_LINE_TARGETS:
    target_elf = join("$BUILD_DIR", "${PROGNAME}.elf")
    target_bin = join("$BUILD_DIR", "${PROGNAME}.bin")
else:
    target_elf = env.BuildProgram()
    target_bin = env.ElfToBin(join("$BUILD_DIR", "${PROGNAME}"), target_elf)
    env.Depends(target_bin, target_elf)

AlwaysBuild(env.Alias("nobuild", target_bin))
target_buildprog = env.Alias("buildprog", target_bin, target_bin)

# ============================================================================
# Upload via Air105 USB bootloader (air105-dfu)
# ============================================================================

upload_protocol = env.subst("$UPLOAD_PROTOCOL") or board.get(
    "upload.protocol", ""
)

upload_actions = []

if upload_protocol == "air105-dfu":
    UPLOADER_SCRIPT = join(PLATFORM_DIR, "tools", "uploader", "upload.py")

    # Use PlatformIO's Python environment
    python_exe = sys.executable

    env.Replace(
        UPLOADER=UPLOADER_SCRIPT,
        UPLOADERFLAGS=["0x1001000"],
        UPLOADCMD='"%s" "%s" "$UPLOAD_PORT" "$BUILD_DIR/${PROGNAME}.bin" 0x1001000'
        % (python_exe, UPLOADER_SCRIPT),
    )

    upload_actions = [
        env.VerboseAction("$UPLOADCMD", "Uploading $SOURCE via Air105 DFU")
    ]

AlwaysBuild(env.Alias("upload", target_bin, upload_actions))

# ============================================================================
# Default target
# ============================================================================

Default([target_buildprog])
