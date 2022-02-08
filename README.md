# Tegra Boot Tools
This repository contains boot-related tools for Tegra platforms,
which augment and/or replace the boot control and bootloader
upgrade tools provided in the L4T BSP for Jetson TX2 and Xavier
platforms.

See the [doc](doc/) directory for descriptions of the tools.

# Building
This package uses CMake 3.8 or later to build.

## Dependencies
This package depends on systemd, libz, and
[tegra-eeprom-tool](https://github.com/OE4T/tegra-eeprom-tool).

For tegra210-based platforms, a configuration file enumerating the
boot partitions, with offsets and sizes, is expected at runtime.
This file should be generated from the flash layout XML file for
the target. See the [tegra-bootpart-config](https://github.com/OE4T/meta-tegra/blob/master/recipes-bsp/tools/tegra-bootpart-config_1.0.bb)
recipe for an example of how to do this.

For all platforms, you need to configure at build time the device
names for the locations of the rootfs and bootloaders, as well as
the target machine name for TNSPEC matching in BUP payloads.

# License
Distributed under license. See the [LICENSE](LICENSE) file for details.

Header files under the [nvidia/t18x](nvidia/t18x) and [nvidia/t19x](nvidia/t19x)
directories are from NVIDIA, with a BSD 3-clause license.  See the individual
header files for details.
