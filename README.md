# Tegra Boot Tools
This repository contains boot-related tools for Tegra platforms,
which augment and/or replace the boot control and bootloader
upgrade tools provided in the L4T BSP for Jetson TX2 and Xavier
platforms.

## tegra-bootinfo
The `tegra-bootinfo` tool provides a simple boot-count mechanism.
The native bootloaders on the TX2 and Xavier have some support for
automatic failover in the event of a boot failure, but they give up
after some number of failover attempts, eventually placing the device
in recovery when the limit is exceeded - not a desirable outcome
if the device runs unattended and is possibly subject to power outages
that can happen at boot time.

The `tega-bootinfo` tool is intended to be run from the initrd on
these systems. It records (in a sector of the eMMC, or in the QSPI
boot flash on Xavier NX development kits) that a boot is in progress,
and notifies the NVIDIA bootloader that the boot was successful (marking
the boot slot successful and setting its priority to 15).
A second invocation of `tegra-bootinfo` should be used at (or close to)
the end of the system startup sequence to record the successful completion
of the start of the OS.

Service unit files (`bootcountcheck.service` for the early boot phase and
`update_bootinfo.service` for the later boot phase) are provided, as is
a `bootcountcheck` script that can be used directly in non-systemd
initramfs environments for the early check.

The tool also supports storage and retrieval of named strings, similar
to U-Boot environment variables. The number of flash sectors allocated
for variable storage is configurable at build time.

On T210 (Jetson TX1 and Nano) systems, U-Boot can be configured provide
the boot count mechanism and appropriate failover behavior.  This tool
will work on those systems as well, however.


## tegra-bootloader-update
This tool can be used to parse a bootloader update (BUP) payload generated
by the Tegra flashing tools, as well as program the appropriate contents
of a BUP payload into the boot partitions. It can replace the
`nv_update_engine` tool provided in the L4T BSP for tegra186/tegra194
platforms, and the `l4t_payload_updater_t210` tool on tegra210
platforms.

### Differences from `nv_update_engine`

* No need for the `/etc/nv_boot_control.conf` file. The TNSPEC used to
  determine boot component compatibility is directly derived from the
  CVM EEPROM contents.
* No `nv_update_verifier` service at boot time. Such a service could
  be implemented, if desired, but for full A/B upgrades that include
  the rootfs, the verify-and-overwrite feature of the L4T update
  verifier is undesirable.
* The tool can be used to completely initialize all boot components,
  provided the BUP payload contains all of them (default BUP generation
  in L4T does **not** do this).
* This tool compares the contents of each boot component from the current
  content before updating and skips unneeded writes, saving update time
  on Xavier NX systems that store boot components in QSPI flash.
* This tool automatically enables A/B redundancy during an update if
  it has not yet been enabled.

### Differences from `l4t_payload_updater_t210`

* Written in C, rather than Python.
* Boot partition information is read from a configuration file at
  runtime, rather than being hard-coded into the tool.
* Automatically handles either SPI flash or eMMC boot partitions,
  without depending on the MACHINE name as the Python tool does.

## tegra-boot-control
This tool replaces the `nvbootctrl` program provided with the L4T BSP,
and can be used to display or modify the boot slots on a tegra186
or tegra194-based platform.

### Differences from `nvbootctrl`

* The interface is different, using option specifiers instead of commands.
  Output from the the `--status` operation uses different formatting
  than `nvbootctrl dump-slots-info`.
* For the `--mark-successful` operation, if the boot slot being marked
  successful is also the current boot slot, the slot priority is
  automatically reset to the highest value (15), rather than requiring
  a separate `--set-active` operation to do this, which ensures that
  the current boot slot remains the default active boot slot on subsequent
  reboots.

# Builds
This package uses GNU Autotools to build.  If you have cloned the repository,
you should use `autoreconf -i` to generate the configure script and other
files needed to configure and build.

## Dependencies
This package depends on systemd, libz, and
[tegra-eeprom-tool](https://github.com/OE4T/tegra-eeprom-tool).

For tegra210-based platforms, a configuration file enumerating the
boot partitions, with offsets and sizes, is expected at runtime.
This file should be generated from the flash layout XML file for
the target.

# License
Distributed under license. See the [LICENSE](LICENSE) file for details.

Header files under the [nvidia/t18x](nvidia/t18x) and [nvidia/t19x](nvidia/t19x)
directories are from NVIDIA, with a BSD 3-clause license.  See the individual
header files for details.
