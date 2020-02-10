dnl
dnl configure.ac - autoconf script for Tegra boot tools
dnl
dnl Copyright (c) 2019, 2020 Matthew Madison
dnl

AC_INIT([tegra-boot-tools], [1.1.0])
AC_DEFINE([TEGRA_BOOT_TOOLS_VERSION_MAJOR], [1], [Major version])
AC_DEFINE([TEGRA_BOOT_TOOLS_VERSION_MINOR], [1], [Minor version])
AC_DEFINE([TEGRA_BOOT_TOOLS_VERSION_MAINT], [0], [Maintenance level])
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

PKG_PROG_PKG_CONFIG([0.29])
PKG_CHECK_MODULES([ZLIB], [zlib])

AS_IF([test "x$with_systemdsystemunitdir" = "xcheck"],
    [with_systemdsystemunitdir=`$PKG_CONFIG --variable systemdsystemunitdir systemd`])
AC_SUBST([systemdsystemunitdir], [$with_systemdsystemunitdir])

AC_CONFIG_FILES(Makefile units/update_bootinfo.service)
AC_PROG_CC
AC_PROG_CC_C_O
AC_OUTPUT
