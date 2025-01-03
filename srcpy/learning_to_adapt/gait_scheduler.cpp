#include "learning_to_adapt/gait_scheduler.hpp"
#include "learning_to_adapt/gait_type.hpp"
#include "learning_to_adapt/gait_data.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

using namespace bio_gait;
namespace py = pybind11;

void bind_gait_scheduler(py::module& m)
{
    py::class_<GaitScheduler>(m, "GaitScheduler")
        .def(py::init<const double, UserCommand&, std::string, const double, const double>())
        .def("gaitData", (const GaitData& (GaitScheduler::*)() const) &GaitScheduler::gaitData)
        .def("gaitData", (GaitData& (GaitScheduler::*)()) &GaitScheduler::gaitData)
        .def("setupSwingFootTrajGen", &GaitScheduler::setupSwingFootTrajGen)
        .def("step", &GaitScheduler::step)
        .def("modifyGait", &GaitScheduler::modifyGait)
        .def("createGait", &GaitScheduler::createGait)
        .def("calcAuxiliaryGaitData", &GaitScheduler::calcAuxiliaryGaitData)
        ;
}
