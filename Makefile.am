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
tmpfilesdir = @tmpfilesdir@

EXTRA_DIST = README.md LICENSE units/finished-booting.target units/update_bootinfo.service.in tmpfiles/tegra-bootinfo.conf

bin_PROGRAMS = tegra-bootloader-update
tegra_bootloader_update_SOURCES = tegra-bootloader-update.c gpt.c gpt.h bup.c bup.h soctype.c soctype.h bct_t18x.c bct_t19x.c bct.h \
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
tegra_bootloader_update_CFLAGS = $(ZLIB_CFLAGS) -I$(srcdir)/nvidia
tegra_bootloader_update_LDADD = $(ZLIB_LIBS)
tegra_bootloader_update_LDFLAGS = $(ZLIB_LDFLAGS)

sbin_PROGRAMS = tegra-bootinfo
tegra_bootinfo_SOURCES = tegra-bootinfo.c
tegra_bootinfo_CFLAGS = $(ZLIB_CFLAGS)
tegra_bootinfo_LDADD = $(ZLIB_LIBS)
tegra_bootinfo_LDFLAGS = $(ZLIB_LDFLAGS)

systemdsystemunit_DATA = units/finished-booting.target units/update_bootinfo.service
tmpfiles_DATA = tmpfiles/tegra-bootinfo.conf
