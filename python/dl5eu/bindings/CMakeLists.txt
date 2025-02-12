# Copyright 2020 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Check if there is C++ code at all
########################################################################
if(NOT dl5eu_sources)
    message(STATUS "No C++ sources... skipping python bindings")
    return()
endif(NOT dl5eu_sources)

########################################################################
# Check for pygccxml
########################################################################
gr_python_check_module_raw("pygccxml" "import pygccxml" PYGCCXML_FOUND)

include(GrPybind)

########################################################################
# Python Bindings
########################################################################

list(APPEND dl5eu_python_files
    dvbt_ofdm_synchronization_python.cc
    dvbt_tps_decoder_python.cc
    dvbt_viterbi_decoder_python.cc
    dvbt_convolutional_deinterleaver_python.cc python_bindings.cc)

gr_pybind_make_oot(dl5eu ../../.. gr::dl5eu "${dl5eu_python_files}")

# copy bindings extension for use in QA test module
add_custom_command(
    TARGET dl5eu_python
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:dl5eu_python>
            ${PROJECT_BINARY_DIR}/test_modules/gnuradio/dl5eu/)

install(
    TARGETS dl5eu_python
    DESTINATION ${GR_PYTHON_DIR}/gnuradio/dl5eu
    COMPONENT pythonapi)
