package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.CommandBase;

public abstract class LedCommand extends CommandBase {
    private final LedStrip ledStrip;

    protected LedCommand(LedStrip ledStrip) {
        this.ledStrip = ledStrip;

        addRequirements(ledStrip);
    }

    @Override
    public void initialize() {
        ledStrip.cancelOverlapping();
    }
}
