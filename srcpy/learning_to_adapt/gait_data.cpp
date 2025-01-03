#include "learning_to_adapt/gait_data.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

using namespace bio_gait;
namespace py = pybind11;

void bind_gait_data(py::module& m)
{
    py::class_<GaitData>(m, "GaitData")
        .def(py::init<>())
        .def("setZero", &GaitData::setZero)
        .def_readwrite("gait_name", &GaitData::gait_name)
        .def_readwrite("current_gait", &GaitData::current_gait)
        .def_readwrite("next_gait", &GaitData::next_gait)
        .def_readwrite("period_time_nominal", &GaitData::period_time_nominal)
        .def_readwrite("initial_phase", &GaitData::initial_phase)
        .def_readwrite("switching_phase_nominal", &GaitData::switching_phase_nominal)
        .def_readwrite("overrideable", &GaitData::overrideable)
        .def_readwrite("gait_enabled", &GaitData::gait_enabled)
        .def_readwrite("period_time", &GaitData::period_time)
        .def_readwrite("time_stance", &GaitData::time_stance)
        .def_readwrite("time_swing", &GaitData::time_swing)
        .def_readwrite("time_stance_remaining", &GaitData::time_stance_remaining)
        .def_readwrite("time_swing_remaining", &GaitData::time_swing_remaining)
        .def_readwrite("switching_phase", &GaitData::switching_phase)
        .def_readwrite("phase_variable", &GaitData::phase_variable)
        .def_readwrite("phase_offset", &GaitData::phase_offset)
        .def_readwrite("phase_scale", &GaitData::phase_scale)
        .def_readwrite("phase_stance", &GaitData::phase_stance)
        .def_readwrite("phase_swing", &GaitData::phase_swing)
        .def_readwrite("contact_state_scheduled", &GaitData::contact_state_scheduled)
        .def_readwrite("contact_state_prev", &GaitData::contact_state_prev)
        .def_readwrite("touchdown_scheduled", &GaitData::touchdown_scheduled)
        .def_readwrite("liftoff_scheduled", &GaitData::liftoff_scheduled)
        .def_readwrite("trans_flag", &GaitData::trans_flag)
        .def_readwrite("des_foot_height", &GaitData::des_foot_height)
        .def_readwrite("ref_foot_x", &GaitData::ref_foot_x)
        .def_readwrite("ref_foot_y", &GaitData::ref_foot_y)
        .def_readwrite("ref_foot_z", &GaitData::ref_foot_z)
        .def_readwrite("FrStab", &GaitData::FrStab)
        .def_readwrite("froudeCmd_", &GaitData::froudeCmd_)
        ;
}
