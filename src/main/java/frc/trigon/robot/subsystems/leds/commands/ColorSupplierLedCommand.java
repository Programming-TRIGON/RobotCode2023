package frc.trigon.robot.subsystems.leds.commands;

import edu.wpi.first.wpilibj.util.Color;
import frc.trigon.robot.subsystems.leds.LedCommand;
import frc.trigon.robot.subsystems.leds.LedStrip;

import java.util.Arrays;
import java.util.function.Supplier;

public class ColorSupplierLedCommand extends LedCommand {
    private final Supplier<Color> colorSupplier;
    /**
     * Constructs a new LedCommand.
     *
     * @param ledStrip the led strip
     */
    protected ColorSupplierLedCommand(LedStrip ledStrip, Supplier<Color> colorSupplier) {
        super(ledStrip);

        this.colorSupplier = colorSupplier;
    }

    @Override
    public void execute() {
        Color[] colors = new Color[getLedStrip().getLength()];
        Arrays.fill(colors, colorSupplier.get());
        setLeds(colors);
    }
}
