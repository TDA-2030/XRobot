#include "pybind11/pybind11.h"
using namespace pybind11::literals;
namespace py = pybind11;

#include "../sdk_common/RMAxis.h"

namespace pybind11 {
    namespace detail {
        template <> struct type_caster<version_t> {
        public:
            PYBIND11_TYPE_CASTER(version_t, _("version"));

            bool load(handle src, bool) 
            {
                value.major = src.attr("major").cast<int32_t>();
                value.minor = src.attr("minor").cast<int32_t>();
                value.build = src.attr("build").cast<int32_t>();
                value.type = src.attr("type").cast<int32_t>();
                return !PyErr_Occurred();
            }

            static handle cast(version_t src, return_value_policy policy, handle parent) 
            {
                py::dict dict;
                dict["major"] = src.major;
                dict["minor"] = src.minor;
                dict["build"] = src.build;
                dict["type"] = src.type;

                return dict.release();
            }
        };

        template <> struct type_caster<command_t> {
        public:
            PYBIND11_TYPE_CASTER(command_t, _("command"));

            bool load(handle src, bool)
            {
                value.type = (command_type_t)src.attr("type").cast<int32_t>();
                value.position = src.attr("position").cast<float>();
                value.velocity = src.attr("velocity").cast<float>();
                value.acceleration = src.attr("acceleration").cast<float>();
                value.deacceleration = src.attr("deacceleration").cast<float>();
                value.band = src.attr("band").cast<float>();
                value.push_force = src.attr("push_force").cast<float>();
                value.push_distance = src.attr("push_distance").cast<float>();
                value.delay = src.attr("delay").cast<int32_t>();
                value.next_command_index = src.attr("next_command_index").cast<int32_t>();
                return !PyErr_Occurred();
            }

            static handle cast(command_t src, return_value_policy policy, handle parent)
            {
                
                py::dict dict;
                dict["type"] = (int32_t)src.type;
                dict["position"] = src.position;
                dict["velocity"] = src.velocity;
                dict["acceleration"] = src.acceleration;
                dict["deacceleration"] = src.deacceleration;
                dict["band"] = src.band;
                dict["push_force"] = src.push_force;
                dict["push_distance"] = src.push_distance;
                dict["delay"] = src.delay;
                dict["next_command_index"] = src.next_command_index;

                return dict.release();
            }
        };
    }
}

PYBIND11_MODULE(motormaster, m) {
    m.def("create_axis_modbus_rtu", &RMAxis::create_rmaxis_modbus_rtu);
    m.def("create_axis_modbus_tcp", &RMAxis::create_rmaxis_modbus_tcp);
    m.def("destroy_axis", &RMAxis::destroy_rmaxis);

    py::class_<RMAxis, std::unique_ptr<RMAxis, py::nodelete>>(m, "axis_modbus")
        .def("get_version", &RMAxis::get_version)
        .def("set_input_signal", &RMAxis::set_input_signal)
        .def("get_output_signal", &RMAxis::get_output_signal)

        .def("config_motion", &RMAxis::config_motion)
        .def("move_to", &RMAxis::move_to)
        .def("go_home", &RMAxis::go_home)
        .def("move_absolute", &RMAxis::move_absolute)
        .def("move_relative", &RMAxis::move_relative)
        .def("push", &RMAxis::push)
        .def("precise_push", &RMAxis::precise_push)

        .def("is_moving", &RMAxis::is_moving)
        .def("is_reached", &RMAxis::is_reached)
        .def("is_push_empty", &RMAxis::is_push_empty)

        .def("set_command", &RMAxis::set_command)
        .def("get_command", &RMAxis::get_command)
        .def("execute_command", &RMAxis::execute_command)
        .def("trig_command", &RMAxis::trig_command)
        .def("load_commands", &RMAxis::load_commands)
        .def("save_commands", &RMAxis::save_commands)

        .def("position", &RMAxis::position)
        .def("velocity", &RMAxis::velocity)
        .def("torque", &RMAxis::torque)
        .def("force_sensor", &RMAxis::force_sensor)
        .def("error_code", &RMAxis::error_code)

        .def("load_parameters", &RMAxis::load_parameters)
        .def("save_parameters", &RMAxis::save_parameters)

        .def("reset_error", &RMAxis::reset_error)
        .def("set_servo_on_off", &RMAxis::set_servo_on_off)
        .def("stop", &RMAxis::stop)
        ;
}