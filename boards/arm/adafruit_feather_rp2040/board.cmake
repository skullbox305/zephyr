# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=RP2040_M0_0" "--speed=4000")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
