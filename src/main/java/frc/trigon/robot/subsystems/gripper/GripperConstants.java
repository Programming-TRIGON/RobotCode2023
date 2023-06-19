package frc.trigon.robot.subsystems.gripper;

public class GripperConstants {
    enum GripperState {
        STOP(0),
        COLLECT(-0.9),
        SLOW_COLLECT(-0.3),
        EJECT(0.43),
        FULL_EJECT(1),
        SLOW_EJECT(0.12),
        HOLD(-0.1);

        final double power;

        GripperState(double power) {
            this.power = power;
        }
    }
}
