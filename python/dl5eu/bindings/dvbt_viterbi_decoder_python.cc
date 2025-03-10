/*
 * Copyright 2025 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/***********************************************************************************/
/* This file is automatically generated using bindtool and can be manually edited  */
/* The following lines can be configured to regenerate this file during cmake      */
/* If manual edits are made, the following tags should be modified accordingly.    */
/* BINDTOOL_GEN_AUTOMATIC(0)                                                       */
/* BINDTOOL_USE_PYGCCXML(0)                                                        */
/* BINDTOOL_HEADER_FILE(dvbt_viterbi_decoder.h)                                        */
/* BINDTOOL_HEADER_FILE_HASH(4816138c51837a3c720ad4c2520f196d)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <gnuradio/dl5eu/dvbt_viterbi_decoder.h>
// pydoc.h is automatically generated in the build directory
#include <dvbt_viterbi_decoder_pydoc.h>

void bind_dvbt_viterbi_decoder(py::module& m)
{

    using dvbt_viterbi_decoder = gr::dl5eu::dvbt_viterbi_decoder;


    py::class_<dvbt_viterbi_decoder,
               gr::block,
               gr::basic_block,
               std::shared_ptr<dvbt_viterbi_decoder>>(
        m, "dvbt_viterbi_decoder", D(dvbt_viterbi_decoder))

        .def(py::init(&dvbt_viterbi_decoder::make),
             py::arg("constellation"),
             py::arg("hierarchy"),
             py::arg("coderate"),
             py::arg("bsize"),
             D(dvbt_viterbi_decoder, make))


        ;
}
