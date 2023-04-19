# tegra-bootinfo

The `tegra-bootinfo` tool uses persistent storage to implement
boot counting and limit checks, and named storage of text strings
similar to U-Boot environment variables.

The native bootloaders on the TX2 and Xavier have some support for
automatic failover in the event of a boot failure, but they give up
after some number of failover attempts, eventually placing the device
in recovery when the limit is exceeded - not a desirable outcome
if the device runs unattended and is possibly subject to power outages
that can happen at boot time.

The `tegra-bootinfo` tool is intended to be run from the initrd on
these systems. It records (in a sector of the eMMC, or in the QSPI
boot flash on non-eMMC-equipped Jetson modules) that a boot is in progress,
and notifies the NVIDIA bootloader (on TX2/Xavier devices) that the boot
was successful by marking the boot slot successful and setting its
boot priority to 15.

A second invocation of `tegra-bootinfo` should be used at (or close to)
the end of the system startup sequence to record the successful completion
of the start of the OS.

systemd service unit files (`bootcountcheck.service` for the early
boot phase and `update_bootinfo.service` for the later boot phase)
are provided, as is  a `bootcountcheck` script that can be used
directly in non-systemd initramfs environments for the early check.

The tool also supports storage and retrieval of named strings, similar
to U-Boot environment variables. The number of flash sectors allocated
for variable storage is configurable at build time. There are patches
for cboot available in [meta-tegra](https://github.com/OE4T/meta-tegra)
that use this mechanism to set the `systemd.machine_id` kernel parameter
based on one of these variables, to provide a persistent machine ID
for systemd when using a read-only root filesystem

The tool may be used on T210 (Jetson TX1 and Nano) systems as well, but
on those platforms it is probably better to configure U-Boot to implement
the boot count mechanism and appropriate failover behavior.
