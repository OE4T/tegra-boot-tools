# tegra-boot-control

This tool replaces the `nvbootctrl` program provided with the L4T BSP,
and can be used to display or modify the boot slots on a tegra186 (TX2)
or tegra194 (Xavier) based platform.

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

A script is included to emulate the `nvbootctrl` command line interface
using this tool.
