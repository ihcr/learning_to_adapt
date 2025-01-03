#include "learning_to_adapt/gait_type.hpp"

#include <pybind11/pybind11.h>

using namespace bio_gait;
namespace py = pybind11;

void bind_gait_type(py::module& m)
{
    py::enum_<GaitType>(m, "GaitType")
        .value("STAND", GaitType::STAND)
        .value("STATIC_WALK", GaitType::STATIC_WALK)
        .value("TROT_WALK", GaitType::TROT_WALK)
        .value("TROT", GaitType::TROT)
        .value("TROT_RUN", GaitType::TROT_RUN)
        .value("PACE", GaitType::PACE)
        .value("BOUND", GaitType::BOUND)
        .value("ROTARY_GALLOP", GaitType::ROTARY_GALLOP)
        .value("TRAVERSE_GALLOP", GaitType::TRAVERSE_GALLOP)
        .value("PRONK", GaitType::PRONK)
        .export_values();
}
