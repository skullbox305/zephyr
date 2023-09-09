#
#  Copyright (c) 2023 Igor Knippenberg <igor.knippenberg@gmail.com>
#
#  SPDX-License-Identifier: Apache-2.0
#
zephyr_library()

zephyr_library_sources(atlas_scientific_oem.c)
zephyr_library_sources_ifdef(CONFIG_ATLAS_SCIENTIFIC_OEM_TRIGGER atlas_scientific_oem_trigger.c)
