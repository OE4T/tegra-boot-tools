# Tegra boot/update tools
#
# Copyright (c) 2019, 2020 Matthew Madison
#

AUTOMAKE_OPTIONS = subdir-objects foreign
ACLOCAL_AMFLAGS = -I m4
SUBDIRS =
DIST_SUBDIRS =

prefix = /usr
exec_prefix = /usr
datadir = $(prefix)/share
sbindir = $(exec_prefix)/sbin
systemdsystemunitdir = @systemdsystemunitdir@

EXTRA_DIST = units/finished-booting.target units/update_bootinfo_service.in

bin_PROGRAMS = tegra-bootloader-update
tegra_bootloader_update_SOURCES = tegra-bootloader-update.c gpt.c bup.c soctype.c bct_t18x.c bct_t19x.c
tegra_bootloader_update_CFLAGS = $(ZLIB_CFLAGS) -I$(srcdir)/nvidia
tegra_bootloader_update_LDADD = $(ZLIB_LIBS)
tegra_bootloader_update_LDFLAGS = $(ZLIB_LDFLAGS)

sbin_PROGRAMS = tegra-bootinfo
tegra_bootinfo_SOURCES = tegra-bootinfo.c
tegra_bootinfo_CFLAGS = $(ZLIB_CFLAGS)
tegra_bootinfo_LDADD = $(ZLIB_LIBS)
tegra_bootinfo_LDFLAGS = $(ZLIB_LDFLAGS)

systemdsystemunit_DATA = units/finished-booting.target units/update_bootinfo.service
