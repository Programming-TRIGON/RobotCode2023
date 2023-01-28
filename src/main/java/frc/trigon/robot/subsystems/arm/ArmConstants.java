package frc.trigon.robot.subsystems.arm;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmConstants {
    public static final int MOTOR_ID1 = 0;
    public static final int MOTOR_ID2 = 0;
    final static WPI_TalonFX MOTOR1 = new WPI_TalonFX(MOTOR_ID1);
    final static WPI_TalonFX MOTOR2 = new WPI_TalonFX(MOTOR_ID2);

    final static int velocity = 0;
    final static int acceleration = 0;
    enum ArmPosition {
        START,
        CONE,
        CUBE;
        public int compareTo() {
            switch (this) {
                case START:
                    return 0;
                case CONE:
                    return 0;
                case CUBE:
                    return 0;
                default:
                    return 0;
            }
        }
    }
    
    final static double
            kP = 0,
            kI = 0,
            kD = 0,
            kF = 0,
            percentOutput = 0;

}



