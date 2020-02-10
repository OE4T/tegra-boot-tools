# Tegra Boot Tools
This repository contains boot-related tools for Tegra platforms.

## tegra-bootinfo
The `tegra-bootinfo` tool provides a simple boot-count mechanism for
Tegra platforms that do not use U-Boot as an intermediate bootloader
Jetson TX2 and Jetson Xavier systems.

The native bootloaders on the TX2 and Xavier have some support for
automatic failover in the event of a boot failure, but they give up
after some number of failover attempts, eventually placing the device
in recovery when the limit is exceeded - not a desirable outcome
if the device runs unattended and is possibly subject to power outages
that can happen at boot time.

The `tega-bootinfo` tool is intended to be run from the initrd on
these systems. It records (in a sector of the eMMC) that a boot is
in progress, and can be accompanied by an invocation of the `nvbootctrl`
tool to notify the NVIDIA bootloader that the boot was successful.
A second invocation of `tegra-bootinfo` should be used at (or close to)
the end of the system startup sequence to record the successful completion
of the start of the OS.

The invocation from a script in the initrd can be checked for a special error
return code which indicates that too many failed boots from the current
rootfs have occurred.  The script can then take action based on that
indication - for example, switching to the other boot slot, or booting
from a recovery partition.

## tegra-bootloader-update
This tool can be used to parse a bootloader update (BUP) payload generated
by the Tegra flashing tools, as well as program the appropriate contents
of a BUP payload into the boot partitions on a Tegra (TX2 or Xavier) device.
Unlike the `nv_update_engine` program provided with the L4T BSP, this tool
can be used during a initial system installation to program both the A and B
boot slots. Note that this tool does not currently support tegra210 (Jetson
TX1/Nano) devices for programming.


# Builds
This package uses GNU Autotools to build.  If you have cloned the repository,
you should use `autoreconf -i` to generate the configure script and other
files needed to configure and build.

# License
Distributed under license. See the [LICENSE](LICENSE) file for details.
