"""
PIO-Air105 — Extra build script for PlatformIO
===============================================
This script configures the build environment to use the local
Arduino framework from the repository root.

@author J.Vovk <jzo132@gmail.com>
@url https://github.com/Jozo132/PIO-Air105
@license MIT
"""

import os
import glob
from os.path import join, isfile, isdir, basename
from SCons.Script import DefaultEnvironment, Import

env = DefaultEnvironment()
Import("env")

# Repository root (two levels up from this test folder)
REPO_ROOT = os.path.abspath(join(env.subst("$PROJECT_DIR"), "..", ".."))
FRAMEWORK_DIR = join(REPO_ROOT, "framework")
VENDOR_DIR = join(REPO_ROOT, "vendor")

# Framework paths
CORES_DIR = join(FRAMEWORK_DIR, "cores", "air105")
VARIANTS_DIR = join(FRAMEWORK_DIR, "variants", "air105_devboard")
SYSTEM_DIR = join(FRAMEWORK_DIR, "system")

# Vendor SDK paths (for CMSIS and peripheral headers)
VENDOR_SDK = join(VENDOR_DIR, "luatos-soc-air105")
VENDOR_BSP = join(VENDOR_SDK, "bsp")

print("=" * 60)
print("PIO-Air105 Arduino Framework")
print("=" * 60)
print(f"  Framework: {FRAMEWORK_DIR}")
print(f"  Cores:     {CORES_DIR}")
print(f"  Variant:   {VARIANTS_DIR}")
print(f"  System:    {SYSTEM_DIR}")
print("=" * 60)

# ============================================================================
# Include paths
# ============================================================================

INCLUDES = [
    CORES_DIR,
    VARIANTS_DIR,
    SYSTEM_DIR,
    # Vendor BSP and CMSIS
    join(VENDOR_BSP, "air105", "chip", "include"),  # air105.h and peripheral headers
    join(VENDOR_BSP, "cmsis", "include"),            # CMSIS core_cm4.h etc.
]

env.Append(CPPPATH=INCLUDES)

# ============================================================================
# Build flags - completely replace defaults for our target
# ============================================================================

# Clear default flags from the STM32 platform
env.Replace(
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
        "-T", join(SYSTEM_DIR, "air105_flash.ld"),
    ],
)

env.Replace(
    CPPDEFINES=[
        "AIR105",
        "ARM_MATH_CM4",
        ("__FPU_PRESENT", "1"),
    ],
)

# ============================================================================
# Framework source files
# ============================================================================

# Collect all framework source files
print("Framework sources:")

# Build system files
env.BuildSources(
    join("$BUILD_DIR", "FrameworkSystem"),
    SYSTEM_DIR,
    src_filter=["+<*.c>", "+<*.s>"]
)
print("  [System] startup_air105.s, system_air105.c")

# Build core files
env.BuildSources(
    join("$BUILD_DIR", "FrameworkArduino"),
    CORES_DIR,
    src_filter=["+<*.c>", "+<*.cpp>"]
)
core_sources = glob.glob(join(CORES_DIR, "*.c")) + glob.glob(join(CORES_DIR, "*.cpp"))
for src in core_sources:
    print(f"  [Core] {basename(src)}")

# Build variant files
env.BuildSources(
    join("$BUILD_DIR", "FrameworkVariant"),
    VARIANTS_DIR,
    src_filter=["+<*.c>", "+<*.cpp>"]
)
variant_sources = glob.glob(join(VARIANTS_DIR, "*.c")) + glob.glob(join(VARIANTS_DIR, "*.cpp"))
for src in variant_sources:
    print(f"  [Variant] {basename(src)}")

print("=" * 60)

# ============================================================================
# Upload configuration (Air105 USB bootloader)
# ============================================================================

UPLOADER_DIR = join(VENDOR_DIR, "air105-uploader")
UPLOADER_SCRIPT = join(UPLOADER_DIR, "upload.py")

# Use PlatformIO's Python environment for upload
PYTHON_EXE = "C:\\Users\\HP\\.platformio\\penv\\Scripts\\python.exe"

env.Replace(
    UPLOADER=f'"{PYTHON_EXE}" "{UPLOADER_SCRIPT}"',
    UPLOADERFLAGS=["0x1001000"],
    UPLOADCMD=f'"{PYTHON_EXE}" "{UPLOADER_SCRIPT}" "$UPLOAD_PORT" "$BUILD_DIR/firmware.bin" 0x1001000',
)

# Generate .bin file from .elf
env.AddPostAction(
    "$BUILD_DIR/firmware.elf",
    env.VerboseAction(
        '"$OBJCOPY" -O binary "$BUILD_DIR/firmware.elf" "$BUILD_DIR/firmware.bin"',
        "Building firmware.bin"
    )
)
