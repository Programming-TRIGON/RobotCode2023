package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.subsystems.gripper.Gripper;

public class ArmCommands {
    private static final Arm ARM = Arm.getInstance();

    public static CommandBase getPlaceConeAtHighNodeCommand() {
        return new SequentialCommandGroup(
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CONE_HIGH_1),
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CONE_HIGH_2),
                Gripper.getInstance().getEjectCommand().alongWith(ARM.getGoToStateCommand(ArmConstants.ArmStates.CLOSED))
        );
    }

    public static CommandBase getPlaceConeAtMediumNodeCommand() {
        return new SequentialCommandGroup(
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CONE_MIDDLE_1).until(ARM::atGoal),
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CONE_MIDDLE_2).until(ARM::atGoal),
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CLOSED).until(ARM::atGoal)
        );
    }

    public static CommandBase getPlaceCubeAtMiddleNodeCommand() {
        return new SequentialCommandGroup(
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CUBE_HIGH_1).alongWith(Gripper.getInstance().getHoldCommand()).until(ARM::atGoal),
                Gripper.getInstance().getEjectCommand().alongWith(ARM.getGoToStateCommand(ArmConstants.ArmStates.CLOSED))
        );
    }
}
