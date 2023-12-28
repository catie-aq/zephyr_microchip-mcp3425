# Copyright (c) CATIE 2023
# SPDX-License-Identifier: Apache-2.0

# When working on a driver in a app, target sources mus be used, instead of "zephyr_library"

#zephyr_library()
#zephyr_library_sources(mcp3425.c)

target_sources(app PRIVATE mcp3425.c)
