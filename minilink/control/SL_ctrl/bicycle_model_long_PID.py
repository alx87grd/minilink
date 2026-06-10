from minilink.control.generic_pid import PID
from minilink.control.motor_map import AccToRearForce, ThrMap
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    DynamicBicycleRearWheelDriveEngine,
)


class PIDLongSL(DiagramSystem):
    def __init__(self, vehicle: DynamicBicycleRearWheelDriveEngine, vx_ref=0.0):
        self.vx_ref = vx_ref

        v_pid = PID(
            Kp=0.8,
            Ki=0.01,
            Kd=0.0,
            cmd_min=-10.0,
            cmd_max=10.0,
            i_min=-5.0,
            i_max=5.0,
            name="Speed PID",
        )

        thr_map = ThrMap(vehicle)
        acc_to_force = AccToRearForce(vehicle)

        super().__init__()
        self.name = "Acceleration PID - DynamicBicycleRearWheelDriveEngine"

        self.add_input_port("targ", dim=1)

        self.add_input_port("V_meas", dim=1)
        self.add_input_port("W_rear_wheel", dim=1)

        self.add_subsystem(acc_to_force, "acc_to_force")
        self.add_subsystem(thr_map, "thr_map")
        self.add_subsystem(v_pid, "v_pid")

        self.connect("input", "targ", "v_pid", "ref")

        self.connect("input", "V_meas", "v_pid", "meas")
        self.connect("input", "W_rear_wheel", "thr_map", "w_rear")

        self.connect("v_pid", "cmd", "acc_to_force", "acc_targ")
        self.connect("acc_to_force", "F_rear", "thr_map", "F_rear")

        self.connect_new_output_port("thr_map", "thr", "thr")


def main():
    vehicle = DynamicBicycleRearWheelDriveEngine()
    ctrl = PIDLongSL(vehicle)
    ctrl.plot_diagram()

    outer = DiagramSystem()

    outer.add_subsystem(ctrl, "longitudinal_controller")

    # Example: connect other outer-level subsystems to the controller inputs
    outer.connect("vehicle", "Vx", "longitudinal_controller", "V_meas")
    outer.connect("vehicle", "W_rear", "longitudinal_controller", "W_rear_wheel")

    outer.connect_new_output_port("longitudinal_controller", "thr", "thr")

    outer.plot_diagram()


if __name__ == "__main__":
    main()
