package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.subsystems.gripper.Gripper;

public class ArmCommands {
    private static final Arm ARM = Arm.getInstance();

    /**
     * @return a command that places a cone at the high node
     */
    public static CommandBase getPlaceConeAtHighNodeCommand() {
        return new SequentialCommandGroup(
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CONE_HIGH_1),
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CONE_HIGH_2),
                Gripper.getInstance().getEjectCommand().alongWith(ARM.getGoToStateCommand(ArmConstants.ArmStates.CLOSED))
        );
    }

    /**
     * @return a command that places a cone at the middle node
     */
    public static CommandBase getPlaceConeAtMiddleNodeCommand() {
        return new SequentialCommandGroup(
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CONE_MIDDLE_1).until(ARM::atGoal),
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CONE_MIDDLE_2).until(ARM::atGoal),
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CLOSED).until(ARM::atGoal)
        );
    }

    /**
     * @return a command that places a cube at the high node
     */
    public static CommandBase getPlaceCubeAtHighNodeCommand() {
        return new SequentialCommandGroup(
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CUBE_HIGH_1).until(ARM::atGoal),
                Gripper.getInstance().getEjectCommand().alongWith(ARM.getGoToStateCommand(ArmConstants.ArmStates.CLOSED))
        );
    }

    /**
     * @return a command that place a cube at the middle node
     */
    public static CommandBase getPlaceCubeAtMiddleNodeCommand() {
        return new SequentialCommandGroup(
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CUBE_MIDDLE_1).alongWith(Gripper.getInstance().getHoldCommand()).until(ARM::atGoal),
                Gripper.getInstance().getEjectCommand().alongWith(ARM.getGoToStateCommand(ArmConstants.ArmStates.CLOSED))
        );
    }

    /**
     * @return a command that places a game piece at the hybrid node
     */
    public static CommandBase getPlaceGamePieceAtHybridNodeCommand() {
        return new SequentialCommandGroup(
                ARM.getGoToStateCommand(ArmConstants.ArmStates.HYBRID_1).until(ARM::atGoal),
                Gripper.getInstance().getEjectCommand().alongWith(ARM.getGoToStateCommand(ArmConstants.ArmStates.CLOSED))
        );
    }
}
