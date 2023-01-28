package frc.trigon.robot.subsystems.arm;


import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final TalonFX
            motor1  = ArmConstants.MOTOR_1,
            motor2 = ArmConstants.MOTOR_2;

    private final static Arm INSTANCE = new Arm();

    public static Arm getInstance() {
        return INSTANCE;
    }

    private Arm() {

    }
}

