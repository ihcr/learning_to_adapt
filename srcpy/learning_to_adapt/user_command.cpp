#include "learning_to_adapt/user_command.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

using namespace bio_gait;
namespace py = pybind11;

void bind_user_command(py::module& m)
{
    py::class_<UserCommand>(m, "UserCommand")
        .def(py::init<>())
        .def_readwrite("gait_type", &UserCommand::gait_type)
        .def_readwrite("gait_override", &UserCommand::gait_override)
        .def_readwrite("gait_period_time", &UserCommand::gait_period_time)
        .def_readwrite("gait_switching_phase", &UserCommand::gait_switching_phase)
        .def_readwrite("gait_step_height", &UserCommand::gait_step_height)
        .def_readwrite("des_base_lin_vel", &UserCommand::des_base_lin_vel)
        .def_readwrite("des_base_ang_vel", &UserCommand::des_base_ang_vel)
        ;
}
