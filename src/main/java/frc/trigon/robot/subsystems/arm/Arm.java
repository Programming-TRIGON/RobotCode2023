package frc.trigon.robot.subsystems.arm;


import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    TalonFX motor1  = ArmConstants.MOTOR1;
    TalonFX motor2 = ArmConstants.MOTOR2;

    private final static Arm INSTANCE = new Arm();

    public static Arm getInstance() {
        return INSTANCE;
    }

    private Arm() {
        motor1.set(TalonFXControlMode.MotionMagic, ArmConstants.position);
        motor2.set(TalonFXControlMode.MotionMagic, ArmConstants.position);

        motor1.config_kP(0, ArmConstants.kP);
        motor1.config_kI(0, ArmConstants.kI);
        motor1.config_kD(0, ArmConstants.kD);
        motor1.config_kF(0, ArmConstants.kF);
        motor1.configPeakOutputForward(ArmConstants.percentOutput);
    }
}

