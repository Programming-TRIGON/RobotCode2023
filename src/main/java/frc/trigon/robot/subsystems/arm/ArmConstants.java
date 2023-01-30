package frc.trigon.robot.subsystems.arm;


import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class ArmConstants {
    public static final int MOTOR_ID1 = 0;
    public static final int MOTOR_ID2 = 1;
    final static WPI_TalonFX MOTOR_1 = new WPI_TalonFX(MOTOR_ID1);
    final static WPI_TalonFX MOTOR_2 = new WPI_TalonFX(MOTOR_ID2);

    final static int velocity = 0;
    final static int acceleration = 0;

    enum ArmPosition {
        CLOSED,
        PICK_UP_CONE,
        PICK_UP_CUBE,
        PLACE_HYBRID_LOW,
        PLACE_HYBRID_MID,
        PLACE_HYBRID_HIGH;
        final double position;

        ArmPosition() {
            this.position = 0;


        }

    }

    final static double
            firstJointKP = 0,
            firstJointKI = 0,
            firstJointKD = 0,
            firstJointKPercentOutput = 0,
            secondJointKP = 0,
            secondJointKI = 0,
            secondJointKD = 0,
            secondJointKPercentOutput = 0;

    static {
        MOTOR_1.set(TalonFXControlMode.MotionMagic, ArmPosition.CLOSED.position);
        MOTOR_2.set(TalonFXControlMode.MotionMagic, ArmPosition.CLOSED.position);

        MOTOR_1.config_kP(0, ArmConstants.firstJointKP);
        MOTOR_1.config_kI(0, ArmConstants.firstJointKI);
        MOTOR_1.config_kD(0, ArmConstants.firstJointKD);
        MOTOR_1.configPeakOutputForward(ArmConstants.firstJointKPercentOutput);
        MOTOR_2.config_kP(0, ArmConstants.secondJointKP);
        MOTOR_2.config_kI(0, ArmConstants.secondJointKI);
        MOTOR_2.config_kD(0, ArmConstants.secondJointKD);
        MOTOR_2.configPeakOutputForward(0);
    }
}



