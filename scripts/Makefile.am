# tegra-boot-tools scripts makefile
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
libdir = $(prefix)/lib

toolsdir = $(datadir)/tegra-boot-tools
toolslibdir = $(toolsdir)/lib
sysconfdir = /etc
systemdsystemunitdir = @systemdsystemunitdir@

EXTRA_DIST = src/tegra-sysinstall.in src/tegra-sysinstall-menu.in \
		lib/tools-common.in lib/menu-common.in \
		lib/finished-booting.target \
		lib/update_bootinfo.service.in

toolslib_DATA = tools-common menu-common
dist_sbin_SCRIPTS = tegra-sysinstall tegra-sysinstall-menu
systemdsystemunit_DATA = finished-booting.target update_bootinfo.service

rewriter = sed -e's,@DATADIR[@],$(datadir),g' \
	    -e's,@TOOLSDIR[@],$(toolsdir),g' \
	    -e's,@TOOLSLIB[@],$(toolslibdir),g' \
	    -e's,@SBINDIR[@],$(sbindir),g'

tegra-sysinstall: src/tegra-sysinstall.in Makefile
	$(rewriter) $< > $@
tegra-sysinstall-menu: src/tegra-sysinstall-menu.in Makefile
	$(rewriter) $< > $@
tools-common: lib/tools-common.in Makefile
	$(rewriter) $< > $@
menu-common: lib/menu-common.in Makefile
	$(rewriter) $< > $@
finished-booting.target: lib/finished-booting.target
	cp $< $@
update_bootinfo.service: lib/update_bootinfo.service.in
	$(rewriter) $< > $@
