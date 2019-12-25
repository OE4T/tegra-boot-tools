# Tegra boot/update tools
#
# Copyright (c) 2019, Matthew Madison
#

AUTOMAKE_OPTIONS = subdir-objects foreign
ACLOCAL_AMFLAGS = -I m4
SUBDIRS =
DIST_SUBDIRS =

prefix = /usr
exec_prefix = /usr
datadir = $(prefix)/share
sbindir = $(exec_prefix)/sbin

bin_PROGRAMS = tegra-bootloader-update
tegra_bootloader_update_SOURCES = tegra-bootloader-update.c gpt.c bup.c
tegra_bootloader_update_CFLAGS = $(ZLIB_CFLAGS)
tegra_bootloader_update_LDADD = $(ZLIB_LIBS)
tegra_bootloader_update_LDFLAGS = $(ZLIB_LDFLAGS)

sbin_PROGRAMS = tegra-bootinfo
tegra_bootinfo_SOURCES = tegra-bootinfo.c
