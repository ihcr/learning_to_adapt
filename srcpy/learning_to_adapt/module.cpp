#include <pybind11/pybind11.h>

namespace py = pybind11;

void bind_user_command(py::module& module);
void bind_gait_type(py::module& module);
void bind_gait_data(py::module& module);
void bind_gait_scheduler(py::module& module);

PYBIND11_MODULE(learning_to_adapt_pywrap, m)
{
    m.doc() = R"pbdoc(
        Python Bindings of the Gait Scheduler
        ---------------------------------------
        .. currentmodule: bio_gait_pywrap
        .. autosummry::
           : toctree: _generate
    )pbdoc";

    bind_user_command(m);
    bind_gait_type(m);
    bind_gait_data(m);
    bind_gait_scheduler(m);

    #ifdef VERSION_INFO
      m.attr("__version__") = VERSION_INFO;
    #else
      m.attr("__version__") = "dev";
    #endif
}
