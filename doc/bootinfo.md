Boot information block
======================

The tegra-bootinfo program needs persistent
storage for the information it tracks - the
boot count, boot-in-progress flag, and any
persistent boot-time variables it stores.

The location of this persistent storage varies
by platform, dependent on the SoC type and
what storage is available at boot time on the
specific module. The differences are mainly
due to differences in the locations and sizes
of the boot partitions.

Two copies of the storage are maintained for
redundancy.

Tegra210 (TX1, Nano)
--------------------

For tegra210 platforms (TX1, Nano) with eMMC, the
primary copy is stored below the VER area in
the second eMMC boot partition (/dev/mmcblk0boot1),
and the secondary copy is stored between the
VER and VER_b areas.

Example layout on t210 (eMMC) with 8Kib (1 base sector
plus 15 extension sectors) per storage copy:

      sector                                           offset (hex)
       7936         +---------------------------------+  3E0000
                    |          VER_b                  |
                    +---------------------------------+
       8047         +---------------------------------+  3EDE00
                    |       extended var store B      |
                    +---------------------------------+
       8063         =       base devinfo copy B       =  3EFE00
       8064         +---------------------------------+  3F0000
                    |          VER                    |
                    +---------------------------------+
       8175         +---------------------------------+  3FDE00
                    |       extended var store A      |
                    +---------------------------------+
       8191         =       base devinfo copy A       =  3FFE00


A similar layout is used for SDcard-based Nanos, but
located in the boot SPI flash (/dev/mtdblock0):

      sector                                           offset (hex)
       7936         +---------------------------------+  3E0000
                    |          VER_b                  |
                    +---------------------------------+
       8047         +---------------------------------+  3EDE00
                    |       extended var store B      |
                    +---------------------------------+
       8063         =       base devinfo copy B       =  3EFE00
       8064         +---------------------------------+  3F0000
                    |          VER                    |
                    +---------------------------------+
       8175         +---------------------------------+  3FDE00
                    |       extended var store A      |
                    +---------------------------------+
       8191        =       base devinfo copy A       =   3FFE00
      

Tegra186 (TX2)
--------------

All TX2s use an eMMC for booting, so the bootinfo
persistent storage is placed in /dev/mmcblk0boot1,
just above the pseudo-GPT used by NVIDIA bootloaders.

Example layout on t186/t194 with 8KiB (1 base sector
plus 15 extension sectors) per storage copy:

      sector                                           offset (hex)
       8124         +---------------------------------+  3F7800
                    |       extended var store B      |
                    +---------------------------------+
       8139         +---------------------------------+  3F9600
                    |       extended var store A      |
                    +---------------------------------+
       8154         =       base devinfo copy B       =  3FB400
       8155         =       base devinfo copy A       =  3FB600
                    ~---------- buffer ---------------~  3FB800
       8159         +---------------------------------+
                    |           pseudo-GPT            |
       8192         +---------------------------------+
      
We leave a 2-sector buffer above the (34 sector)
pseudo-GPT for this layout.


Tegra194 (Xavier)
-----------------

The AGX Xavier module uses an eMMC. The Xavier NX
modules use a SPI flash, but the Xavier NX SKU 0001
modules also contain an eMMC (the boot partitions
for which are otherwise unused). On these platforms,
we use the same layout as described above for the
TX2 platforms.

Xavier NX SKU 0000 modules do not have an eMMC,
so the bootinfo storage is located on the SPI
flash.  The erase block size for the SPI flash
part used on those modules is 64KiB in size,
so the areas are chosen to avoid erasing the
GPT during bootinfo updates (which could render
the device unbootable if power is cut during the
update) and to avoid overlap of the two areas,
so they don't both get erased during an update
to one area.

       sector                                          offset (hex)
       65152        +---------------------------------+  1FD0000
                    |          VER_b                  |
                    +---------------------------------+
                    ...
       65264        +---------------------------------+  1FDE000
                    |       extended var store B      |
                    +---------------------------------+
       65279        =       base devinfo copy B       =  1FDFE00
                    ~------buffer to 64KiB boundary---~
       65392        +---------------------------------+  1FEE000
                    |       extended var store A      |
                    +---------------------------------+
       65407        =       base devinfo copy A       =  1FEFE00
                    ~------buffer to 64KiB boundary---~
       65503        +---------------------------------+  1FFBE00
                    |           pseudo-GPT            |
       65536        +---------------------------------+  2000000
      

Storage layout notes
--------------------

Note that 4 bytes are reserved at the end of
each of the extended storage blocks for a CRC
checksum for the block, which is maintained
separately from the CRC checksum for the base
block for compatibility with older versions of
this tool that did not support the extensions.
