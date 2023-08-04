package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmConstants {
    public static final double SECOND_JOINT_GEAR_RATIO = 156.6862745098039; // 163.6626506024096
    static final double MINIMUM_END_EFFECTOR_HEIGHT = 20;
    public static final double
            FIRST_JOINT_HEIGHT = 101.91482,
            FIRST_JOINT_LENGTH = 85.88525,
            SECOND_JOINT_LENGTH = 55;

    static final Transform2d
            FIRST_JOINT_TO_SECOND_JOINT = new Transform2d(new Translation2d(FIRST_JOINT_LENGTH, 0), Rotation2d.fromDegrees(0)),
            SECOND_JOINT_TO_END_EFFECTOR = new Transform2d(new Translation2d(SECOND_JOINT_LENGTH, 0), Rotation2d.fromDegrees(0));

    static final double RETRACTED_DEGREES = 130;

    static final double
            DESCEND_PROFILE_COMPLETION_PERCENTAGE = 1,
            RISE_PROFILE_COMPLETION_PERCENTAGE = 0.7;

    static final double
            FIRST_JOINT_TOLERANCE = 6,
            FIRST_JOINT_VELOCITY_TOLERANCE = 900,
            SECOND_JOINT_TOLERANCE = 4,
            SECOND_JOINT_VELOCITY_TOLERANCE = 900;

    static final boolean DEFAULT_NEUTRAL_MODE = true;

    private static final double
            FIRST_JOINT_MAX_SPEED_DEGREES_PER_SECOND = 200,
            SECOND_JOINT_MAX_SPEED_DEGREES_PER_SECOND = 720;

    private static final double
            FIRST_JOINT_MAX_ACCELERATION_DEGREES_PER_SECOND_SQUARED = 975.2,
            SECOND_JOINT_MAX_ACCELERATION_DEGREES_PER_SECOND_SQUARED = 180;

    static final TrapezoidProfile.Constraints
            FIRST_JOINT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                    FIRST_JOINT_MAX_SPEED_DEGREES_PER_SECOND,
                    FIRST_JOINT_MAX_ACCELERATION_DEGREES_PER_SECOND_SQUARED
            ),
            SECOND_JOINT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                    SECOND_JOINT_MAX_SPEED_DEGREES_PER_SECOND,
                    SECOND_JOINT_MAX_ACCELERATION_DEGREES_PER_SECOND_SQUARED
            );

    public static final double
            FIRST_JOINT_CLOSED = -79,
            SECOND_JOINT_CLOSED = 156;

    static final Mechanism2d ARM_MECHANISM = new Mechanism2d(
            (FIRST_JOINT_LENGTH + SECOND_JOINT_LENGTH) * 2 / 100,
            (FIRST_JOINT_LENGTH + SECOND_JOINT_LENGTH) * 2 / 100
    );
    private static final MechanismRoot2d ARM_PIVOT_ROOT = ARM_MECHANISM.getRoot(
            "ArmPivot",
            (FIRST_JOINT_LENGTH + SECOND_JOINT_LENGTH) / 100,
            FIRST_JOINT_HEIGHT / 100
    );

    static final MechanismLigament2d
            TARGET_FIRST_JOINT_LIGAMENT = ARM_PIVOT_ROOT.append(
                    new MechanismLigament2d("TargetFirstJoint", FIRST_JOINT_LENGTH /100, FIRST_JOINT_CLOSED, 10, new Color8Bit(Color.kGray))
            ),
            TARGET_SECOND_JOINT_LIGAMENT = TARGET_FIRST_JOINT_LIGAMENT.append(
                    new MechanismLigament2d("TargetSecondJoint", SECOND_JOINT_LENGTH / 100, SECOND_JOINT_CLOSED, 8, new Color8Bit(Color.kGray))
            ),
            FIRST_JOINT_LIGAMENT = ARM_PIVOT_ROOT.append(
                new MechanismLigament2d("FirstJoint", FIRST_JOINT_LENGTH /100, FIRST_JOINT_CLOSED, 10, new Color8Bit(Color.kRed))
            ),
            SECOND_JOINT_LIGAMENT = FIRST_JOINT_LIGAMENT.append(
                new MechanismLigament2d("SecondJoint", SECOND_JOINT_LENGTH / 100, SECOND_JOINT_CLOSED, 8, new Color8Bit(Color.kBlue))
            );

    public enum ArmStates {
        CLOSED(FIRST_JOINT_CLOSED, SECOND_JOINT_CLOSED),
        CLOSED_COLLECTING(FIRST_JOINT_CLOSED, 86.3),
        CLOSED_COLLECTING_STANDING_CONE(FIRST_JOINT_CLOSED, 99),
        CONE_FEEDER(FIRST_JOINT_CLOSED, 143),
        CUBE_MIDDLE(-51, 117),
        CUBE_HIGH(-5, 35),
        CONE_HYBRID(FIRST_JOINT_CLOSED, SECOND_JOINT_CLOSED),
        CUBE_HYBRID(FIRST_JOINT_CLOSED, SECOND_JOINT_CLOSED),
        CONE_MIDDLE_1(-30, 102),
        CONE_MIDDLE_2(-49, 102),
        CONE_HIGH(22, -19),
        AUTO_CONE_HIGH(24, -19);

        public final double firstMotorPosition, secondMotorPosition;

        ArmStates(double firstMotorPosition, double secondMotorPosition) {
            this.firstMotorPosition = firstMotorPosition;
            this.secondMotorPosition = secondMotorPosition;
        }
    }
}
