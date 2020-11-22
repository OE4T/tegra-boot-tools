dnl
dnl configure.ac - autoconf script for Tegra boot tools
dnl
dnl Copyright (c) 2019, 2020 Matthew Madison
dnl

AC_INIT([tegra-boot-tools], [1.6.2])
AC_DEFINE([TEGRA_BOOT_TOOLS_VERSION_MAJOR], [1], [Major version])
AC_DEFINE([TEGRA_BOOT_TOOLS_VERSION_MINOR], [6], [Minor version])
AC_DEFINE([TEGRA_BOOT_TOOLS_VERSION_MAINT], [2], [Maintenance level])
AM_INIT_AUTOMAKE([subdir-objects foreign])
AM_SILENT_RULES([yes])
AC_COPYRIGHT([
Copyright (c) 2019, 2020 Matthew Madison
])
AC_CONFIG_MACRO_DIR([m4])

AC_PREREQ([2.69])
LT_PREREQ([2.4])
LT_INIT([])

AC_CANONICAL_HOST
AC_PROG_INSTALL

AC_ARG_WITH([systemdsystemunitdir],
    AS_HELP_STRING([--with-systemdsystemunitdir=DIR], [location for systemd service files]),
    [], [with_systemdsystemunitdir=check])
AC_ARG_WITH([tmpfilesdir],
    AS_HELP_STRING([--with-tmpfilesdir=DIR], [location for systemd-tmpfiles config files]),
    [], [with_tmpfilesdir=check])
AC_ARG_WITH([group],
    AS_HELP_STRING([--with-group=GROUP], [group owner for lock directory]),
    [], [with_group=root])
AC_SUBST([LKDIRGROUP], [$with_group])
AC_ARG_WITH([machine-name],
    AS_HELP_STRING([--with-machine-name=STR], [machine configuration name]),
    [], [with_machine_name=])
AC_SUBST([MACHINE], [$with_machine_name])
AC_ARG_WITH([rootfsdev],
    AS_HELP_STRING([--with-rootfsdev=STR], [device name for booting rootfs (without /dev)]),
    [], [with_rootfsdev=mmcblk0p1])
AC_SUBST([ROOTFSDEV], [$with_rootfsdev])
AC_ARG_WITH([bootdev],
    AS_HELP_STRING([--with-bootdev=STR], [device name for boot partitions]),
    [], [with_bootdev=/dev/mmcblk0boot0])
AC_DEFINE_UNQUOTED([OTABOOTDEV], ["$with_bootdev"], [Device where boot partitions are stored])
AC_ARG_WITH([gptdev],
    AS_HELP_STRING([--with-gptdev=STR], [device name for boot partition table]),
    [], [with_gptdev=/dev/mmcblk0boot1])
AC_DEFINE_UNQUOTED([OTAGPTDEV], ["$with_gptdev"], [Device where pseudo-GPT for boot partitions is stored])
AC_ARG_WITH([extended-sector-count],
    AS_HELP_STRING([--with-extended-sector-count=N], [number of 512-byte sectors for boot variable storage]),
    [], [with_extended_sector_count=1])
AC_DEFINE_UNQUOTED([EXTENSION_SECTOR_COUNT], [$with_extended_sector_count], [Number of extension sectors])

PKG_PROG_PKG_CONFIG([0.29])
PKG_CHECK_MODULES([ZLIB], [zlib])
PKG_CHECK_MODULES([TEGRA_EEPROM], [tegra-eeprom])

AS_IF([test "x$with_systemdsystemunitdir" = "xcheck"],
    [with_systemdsystemunitdir=`$PKG_CONFIG --variable systemdsystemunitdir systemd`])
AC_SUBST([systemdsystemunitdir], [$with_systemdsystemunitdir])
AS_IF([test "x$with_tmpfilesdir" = "xcheck"],
    [with_tmpfilesdir=`$PKG_CONFIG --variable tmpfilesdir systemd`])
AC_SUBST([tmpfilesdir], [$with_tmpfilesdir])

AS_IF([test "$with_gptdev" = "$with_bootdev"],
    [bootdevs="dev-`echo $with_gptdev | cut -d/ -f3`.device"],
    [bootdevs="dev-`echo $with_bootdev | cut -d/ -f3`.device dev-`echo $with_gptdev | cut -d/ -f3`.device"])
AC_SUBST([BOOTDEVS], [$bootdevs])

AC_CONFIG_FILES([Makefile tegra-boot-tools.pc config-files/machine-name.conf config-files/rootfsdev.conf config-files/tegra-bootinfo.conf])
AC_CONFIG_HEADERS([config.h])
AC_PROG_CC
AC_PROG_CC_C_O
AC_OUTPUT
