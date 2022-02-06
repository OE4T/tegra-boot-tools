# SPDX-License-Identifier: MIT
# Copyright (c) 2021, Matthew Madison

cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0048 NEW)

project(tegra_boot_tools LANGUAGES C VERSION 2.5.99)

include(GNUInstallDirs)

set(CMAKE_C_STANDARD 11)

set(SYSTEMD_SYSTEM_UNITDIR "" CACHE PATH "Location for systemd service files")
set(TMPFILESDIR "" CACHE PATH "Location for systemd-tmpfiles config files")
set(LOCK_DIR_GROUP "root" CACHE STRING "Group owner for lock directory")
set(MACHINE "" CACHE STRING "Machine configuration name")
set(ROOTFS_DEVICE "mmcblk0p1" CACHE STRING "Device name for booting rootfs, without /dev")
set(BOOT_DEVICE "/dev/mmcblk0boot0" CACHE PATH "Device where boot partitions are stored")
set(GPT_DEVICE "/dev/mmcblk0boot1" CACHE PATH "Device where pseudo-GPT for boot partitions is stored")
set(EXTENSION_SECTOR_COUNT "15" CACHE STRING "Number of extra 512-byte sectors for boot variable storage")

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZLIB REQUIRED IMPORTED_TARGET zlib)
pkg_check_modules(UUID REQUIRED IMPORTED_TARGET uuid)
pkg_check_modules(TEGRA_EEPROM REQUIRED IMPORTED_TARGET tegra-eeprom)
pkg_check_modules(SYSTEMD REQUIRED IMPORTED_TARGET systemd)

if("${SYSTEMD_SYSTEMD_UNITDIR}" STREQUAL "")
  pkg_get_variable(SYSTEMD_SYSTEM_UNITDIR systemd systemdsystemunitdir)
endif()
if("${TMPFILESDIR}" STREQUAL "")
  pkg_get_variable(TMPFILESDIR systemd tmpfilesdir)
endif()

string(REGEX REPLACE "^/dev/(.*)" "\\1" BOOTDEV_BASE "${BOOT_DEVICE}")
if("${BOOT_DEVICE}" STREQUAL "${GPT_DEVICE}")
  set(BOOTDEVS "dev-${BOOTDEV_BASE}.device")
else()
  string(REGEX REPLACE "^/dev/(.*)" "\\1" GPTDEV_BASE "${GPT_DEVICE}")
  set(BOOTDEVS "dev-${BOOTDEV_BASE}.device dev-${GPTDEV_BASE}.device")
endif()

configure_file(config.h.in config.h @ONLY)
configure_file(tegra-boot-tools.pc.in tegra-boot-tools.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/tegra-boot-tools.pc DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")

configure_file("config-files/machine-name.conf.in" "config-files/machine-name.conf" @ONLY)
configure_file("config-files/rootfsdev.conf.in" "config-files/rootfsdev.conf" @ONLY)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/config-files/machine-name.conf
  ${CMAKE_CURRENT_BINARY_DIR}/config-files/rootfsdev.conf
  DESTINATION "${CMAKE_INSTALL_DATADIR}/tegra-boot-tools")

configure_file(config-files/update_bootinfo.service.in config-files/update_bootinfo.service @ONLY)
configure_file(config-files/bootcountcheck.service.in config-files/bootcountcheck.service @ONLY)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/config-files/update_bootinfo.service
  ${CMAKE_CURRENT_BINARY_DIR}/config-files/bootcountcheck.service
  DESTINATION "${SYSTEMD_SYSTEM_UNITDIR}")

configure_file(config-files/tegra-bootinfo.conf.in config-files/tegra-bootinfo.conf @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/config-files/tegra-bootinfo.conf DESTINATION "${TMPFILESDIR}")

add_library(tegra-boot-tools SHARED
  smd.c smd.h gpt.c gpt.h bup.c bup.h ver.c ver.h posix-crc32.c posix-crc32.h util.c util.h bootinfo.c bootinfo.h)
set_target_properties(tegra-boot-tools PROPERTIES
  VERSION 1.0.0
  SOVERSION 1)
target_include_directories(tegra-boot-tools PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/nvidia)
target_link_libraries(tegra-boot-tools PUBLIC PkgConfig::ZLIB PkgConfig::UUID PkgConfig::TEGRA_EEPROM)
target_compile_definitions(tegra-boot-tools PUBLIC "CONFIGPATH=${CMAKE_INSTALL_FULL_DATADIR}/tegra-boot-tools")
install(TARGETS tegra-boot-tools LIBRARY)

add_executable(tegra-bootloader-update
  tegra-bootloader-update.c
  bct_t18x.c
  bct_t19x.c
  bct_t21x.c
  bct.h
  nvidia/nvcommon.h
  nvidia/t18x/arse0.h
  nvidia/t18x/nvboot_crypto_signatures.h
  nvidia/t18x/nvboot_se_rsa.h
  nvidia/t18x/nvboot_section_defs.h
  nvidia/t18x/nvboot_crypto_sha_param.h
  nvidia/t18x/nvboot_sdmmc_param.h
  nvidia/t18x/nvboot_se_aes.h
  nvidia/t18x/nvboot_crypto_aes_param.h
  nvidia/t18x/nvboot_ufs_param.h
  nvidia/t18x/nvboot_crypto_param.h
  nvidia/t18x/nvboot_bct.h
  nvidia/t18x/nvboot_crypto_ecc_param.h
  nvidia/t18x/nvboot_devparams.h
  nvidia/t18x/nvboot_sdram_param.h
  nvidia/t18x/nvboot_sdram_param_generated.h
  nvidia/t18x/nvboot_util_int.h
  nvidia/t18x/nvboot_spi_flash_param.h
  nvidia/t18x/nvboot_crypto_se_param.h
  nvidia/t18x/nvboot_fuse.h
  nvidia/t18x/nvboot_usb3_param.h
  nvidia/t18x/nvboot_sata_param.h
  nvidia/t18x/nvboot_hash.h
  nvidia/t18x/nvboot_se_defs.h
  nvidia/t18x/nvboot_error.h
  nvidia/t18x/nvboot_config.h
  nvidia/t18x/nvboot_crypto_rsa_param.h
  nvidia/t19x/arse0.h
  nvidia/t19x/nvboot_crypto_signatures.h
  nvidia/t19x/nvboot_crypto_public_keystore.h
  nvidia/t19x/nvboot_se_rsa.h
  nvidia/t19x/nvboot_crypto_eddsa_param.h
  nvidia/t19x/nvboot_section_defs.h
  nvidia/t19x/nvboot_crypto_sha_param.h
  nvidia/t19x/nvboot_sdmmc_param.h
  nvidia/t19x/nvboot_se_aes.h
  nvidia/t19x/nvboot_crypto_aes_param.h
  nvidia/t19x/nvboot_ufs_param.h
  nvidia/t19x/nvboot_crypto_param.h
  nvidia/t19x/nvboot_bct.h
  nvidia/t19x/nvboot_crypto_ecc_param.h
  nvidia/t19x/nvboot_devparams.h
  nvidia/t19x/nvboot_sdram_param.h
  nvidia/t19x/nvboot_sdram_param_generated.h
  nvidia/t19x/nvboot_util_int.h
  nvidia/t19x/nvboot_spi_flash_param.h
  nvidia/t19x/nvboot_crypto_se_param.h
  nvidia/t19x/nvboot_fuse.h
  nvidia/t19x/nvboot_sata_param.h
  nvidia/t19x/nvboot_hash.h
  nvidia/t19x/nvboot_se_defs.h
  nvidia/t19x/nvboot_error.h
  nvidia/t19x/nvboot_config.h
  nvidia/t19x/nvboot_crypto_rsa_param.h
  nvidia/t19x/nvboot_boot_component.h)
target_include_directories(tegra-bootloader-update PRIVATE nvidia ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(tegra-bootloader-update PUBLIC tegra-boot-tools PkgConfig::TEGRA_EEPROM)

add_executable(tegra-boot-control tegra-boot-control.c)
target_include_directories(tegra-boot-control PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(tegra-boot-control PUBLIC tegra-boot-tools PkgConfig::TEGRA_EEPROM)

add_executable(tegra-bootinfo tegra-bootinfo.c)
target_include_directories(tegra-bootinfo PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(tegra-bootinfo PUBLIC tegra-boot-tools PkgConfig::ZLIB PkgConfig::TEGRA_EEPROM)

install(TARGETS tegra-boot-tools tegra-bootloader-update tegra-boot-control tegra-bootinfo RUNTIME)
install(PROGRAMS scripts/bootcountcheck scripts/nvbootctrl scripts/nv_update_engine TYPE SBIN)
