"""Add the STM32Cube FreeRTOS kernel to the Arduino PlatformIO build."""

import os

Import("env")


freertos_package = env.PioPlatform().get_package_dir(
    "framework-stm32cubef4"
)
if not freertos_package:
    raise RuntimeError("framework-stm32cubef4 is required for FreeRTOS")

freertos_source = os.path.join(
    freertos_package, "Middlewares", "Third_Party", "FreeRTOS", "Source"
)
freertos_include = os.path.join(freertos_source, "include")
freertos_port = os.path.join(freertos_source, "portable", "GCC", "ARM_CM4F")

# micro_ros_platformio invokes its CMake/colcon builder with PlatformIO's
# package virtualenv.  Keep that interpreter aligned with the one used by the
# pre-build dependency installation instead of relying on the shell's Python.
platformio_python = env.subst("$PYTHONEXE")
platformio_virtualenv_python = os.path.join(
    env.subst("$PROJECT_CORE_DIR"), "penv", "bin", "python"
)
if os.path.exists(platformio_virtualenv_python):
    env["PYTHONEXE"] = platformio_virtualenv_python
    platformio_python = platformio_virtualenv_python
os.environ["PATH"] = os.path.dirname(platformio_python) + os.pathsep + os.environ[
    "PATH"
]

env.Append(CPPPATH=[freertos_include, freertos_port])
env.Append(CPPPATH=[os.path.join(env.subst("$PROJECT_DIR"), "include")])
env.Append(CPPDEFINES=["USE_FreeRTOS_HEAP_4"])
env.BuildSources(
    os.path.join("$BUILD_DIR", "FreeRTOSKernel"),
    freertos_source,
    src_filter=[
        "+<tasks.c>",
        "+<queue.c>",
        "+<list.c>",
        "+<portable/GCC/ARM_CM4F/port.c>",
        "+<portable/MemMang/heap_4.c>",
    ],
)