# Tegra boot/update tools
#
# Copyright (c) 2019-2021, Matthew Madison
#

AUTOMAKE_OPTIONS = subdir-objects foreign
ACLOCAL_AMFLAGS = -I m4
SUBDIRS =
DIST_SUBDIRS =

systemdsystemunitdir = @systemdsystemunitdir@
tmpfilesdir = @tmpfilesdir@
tbtdir = $(datadir)/tegra-boot-tools
tbtincdir = $(includedir)/tegra-boot-tools
pkgconfigdir = $(libdir)/pkgconfig

EXTRA_DIST = README.md LICENSE config-files/update_bootinfo.service.in config-files/bootcountcheck.service.in

# Needed for performing substitutions in systemd service units,
# which can't be done by AC_CONFIG_FILES.
do_subst = sed -e's,[@]bindir[@],$(bindir),g' \
               -e's,[@]sbindir[@],$(sbindir),g' \
               -e's,[@]sysconfdir[@],$(sysconfdir),g' \
               -e's,[@]BOOTDEVS[@],$(BOOTDEVS),g'

lib_LTLIBRARIES = libtegra-boot-tools.la
libtegra_boot_tools_la_SOURCES = smd.c smd.h gpt.c gpt.h bup.c bup.h ver.c ver.h posix-crc32.c posix-crc32.h util.c util.h
libtegra_boot_tools_la_CFLAGS = $(ZLIB_CFLAGS) $(UUID_CFLAGS) $(TEGRA_EEPROM_CFLAGS)
libtegra_boot_tools_la_CPPFLAGS = -DCONFIGPATH="$(tbtdir)"
libtegra_boot_tools_la_LIBADD = $(ZLIB_LIBS) $(UUID_LIBS) $(TEGRA_EEPROM_LIBS)
libtegra_boot_tools_la_LDFLAGS = $(ZLIB_LDFLAGS) $(UUID_LIBS) $(TEGRA_EEPROM_LDFLAGS)
tbtinc_HEADERS = smd.h gpt.h bup.h
pkgconfig_DATA = tegra-boot-tools.pc

bin_PROGRAMS = tegra-bootloader-update tegra-boot-control tegra-bootinfo
tegra_bootloader_update_SOURCES = tegra-bootloader-update.c bct_t18x.c bct_t19x.c bct_t21x.c bct.h \
    nvidia/nvcommon.h \
    nvidia/t18x/arse0.h \
    nvidia/t18x/nvboot_crypto_signatures.h \
    nvidia/t18x/nvboot_se_rsa.h \
    nvidia/t18x/nvboot_section_defs.h \
    nvidia/t18x/nvboot_crypto_sha_param.h \
    nvidia/t18x/nvboot_sdmmc_param.h \
    nvidia/t18x/nvboot_se_aes.h \
    nvidia/t18x/nvboot_crypto_aes_param.h \
    nvidia/t18x/nvboot_ufs_param.h \
    nvidia/t18x/nvboot_crypto_param.h \
    nvidia/t18x/nvboot_bct.h \
    nvidia/t18x/nvboot_crypto_ecc_param.h \
    nvidia/t18x/nvboot_devparams.h \
    nvidia/t18x/nvboot_sdram_param.h \
    nvidia/t18x/nvboot_sdram_param_generated.h \
    nvidia/t18x/nvboot_util_int.h \
    nvidia/t18x/nvboot_spi_flash_param.h \
    nvidia/t18x/nvboot_crypto_se_param.h \
    nvidia/t18x/nvboot_fuse.h \
    nvidia/t18x/nvboot_usb3_param.h \
    nvidia/t18x/nvboot_sata_param.h \
    nvidia/t18x/nvboot_hash.h \
    nvidia/t18x/nvboot_se_defs.h \
    nvidia/t18x/nvboot_osc.h \
    nvidia/t18x/nvboot_error.h \
    nvidia/t18x/nvboot_config.h \
    nvidia/t18x/nvboot_crypto_rsa_param.h \
    nvidia/t19x/arse0.h \
    nvidia/t19x/nvboot_crypto_signatures.h \
    nvidia/t19x/nvboot_crypto_public_keystore.h \
    nvidia/t19x/nvboot_se_rsa.h \
    nvidia/t19x/nvboot_crypto_eddsa_param.h \
    nvidia/t19x/nvboot_section_defs.h \
    nvidia/t19x/nvboot_crypto_sha_param.h \
    nvidia/t19x/nvboot_sdmmc_param.h \
    nvidia/t19x/nvboot_se_aes.h \
    nvidia/t19x/nvboot_crypto_aes_param.h \
    nvidia/t19x/nvboot_ufs_param.h \
    nvidia/t19x/nvboot_crypto_param.h \
    nvidia/t19x/nvboot_bct.h \
    nvidia/t19x/nvboot_crypto_ecc_param.h \
    nvidia/t19x/nvboot_devparams.h \
    nvidia/t19x/nvboot_sdram_param.h \
    nvidia/t19x/nvboot_sdram_param_generated.h \
    nvidia/t19x/nvboot_util_int.h \
    nvidia/t19x/nvboot_spi_flash_param.h \
    nvidia/t19x/nvboot_crypto_se_param.h \
    nvidia/t19x/nvboot_fuse.h \
    nvidia/t19x/nvboot_sata_param.h \
    nvidia/t19x/nvboot_hash.h \
    nvidia/t19x/nvboot_se_defs.h \
    nvidia/t19x/nvboot_error.h \
    nvidia/t19x/nvboot_config.h \
    nvidia/t19x/nvboot_crypto_rsa_param.h \
    nvidia/t19x/nvboot_boot_component.h
tegra_bootloader_update_CFLAGS = $(TEGRA_EEPROM_CFLAGS) -I$(srcdir)/nvidia
tegra_bootloader_update_LDADD = libtegra-boot-tools.la $(TEGRA_EEPROM_LIBS)
tegra_bootloader_update_LDFLAGS = $(TEGRA_EEPROM_LDFLAGS)

tegra_boot_control_SOURCES = tegra-boot-control.c
tegra_boot_control_CFLAGS = $(TEGRA_EEPROM_CFLAGS)
tegra_boot_control_LDADD = libtegra-boot-tools.la $(TEGRA_EEPROM_LIBS)
tegra_boot_control_LDFLAGS = $(TEGRA_EEPROM_LDFLAGS)

tegra_bootinfo_SOURCES = tegra-bootinfo.c
tegra_bootinfo_CFLAGS = $(ZLIB_CFLAGS) $(TEGRA_EEPROM_CFLAGS)
tegra_bootinfo_LDADD = libtegra-boot-tools.la $(ZLIB_LIBS) $(TEGRA_EEPROM_LIBS)
tegra_bootinfo_LDFLAGS = -static $(ZLIB_LDFLAGS) $(TEGRA_EEPROM_LDFLAGS)

systemdsystemunit_DATA = config-files/update_bootinfo.service config-files/bootcountcheck.service

config-files/update_bootinfo.service: config-files/update_bootinfo.service.in Makefile
	$(do_subst) < $(srcdir)/config-files/update_bootinfo.service.in > config-files/update_bootinfo.service
config-files/bootcountcheck.service: config-files/bootcountcheck.service.in Makefile
	$(do_subst) < $(srcdir)/config-files/bootcountcheck.service.in > config-files/bootcountcheck.service

tmpfiles_DATA = config-files/tegra-bootinfo.conf
tbt_DATA = config-files/machine-name.conf config-files/rootfsdev.conf

dist_sbin_SCRIPTS = scripts/bootcountcheck scripts/nvbootctrl scripts/nv_update_engine

CLEANFILES = $(systemdsystemunit_DATA)
