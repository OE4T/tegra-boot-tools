#ifndef util_h_included
#define util_h_included
/* Copyright (c) 2021, Matthew Madison */

#include <stdbool.h>
bool set_bootdev_writeable_status(const char *bootdev, bool make_writeble);
bool partition_should_be_present(const char *partname);

#endif /* util_h_included */
