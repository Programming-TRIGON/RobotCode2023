package frc.trigon.robot.subsystems.arm;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmConstants {
    public static final int MOTOR_ID1 = 0;
    public static final int MOTOR_ID2 = 0;
    final static WPI_TalonFX MOTOR1 = new WPI_TalonFX(MOTOR_ID1);
    final static WPI_TalonFX MOTOR2 = new WPI_TalonFX(MOTOR_ID2);

}



