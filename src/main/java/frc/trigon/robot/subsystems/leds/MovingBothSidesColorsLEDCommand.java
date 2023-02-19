package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MovingBothSidesColorsLEDCommand extends LedCommand {
    Color backgroundColor;
    Color primeColor;
    double cycleTime;
    int amountOfMovingLeds;
    private final LedStrip ledStrip;


    public MovingBothSidesColorsLEDCommand(Color backgroundColor, Color primeColor, double cycleTime, int amountOfMovingLeds, LedStrip ledStrip){
        super(ledStrip);
        this.backgroundColor = backgroundColor;
        this.primeColor = primeColor;
        this.cycleTime = cycleTime;
        this.amountOfMovingLeds = amountOfMovingLeds -1;
        this.ledStrip = ledStrip;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        Color[] colors = new Color[ledStrip.getLength()];
        int firstInMovingRange = (int) ((Timer.getFPGATimestamp() / cycleTime) * 2) % ledStrip.getLength();
        int lastInMovingRange = (firstInMovingRange + amountOfMovingLeds) % ledStrip.getLength();
        defineColors(colors, firstInMovingRange, lastInMovingRange);
        ledStrip.setLedsColors(colors);

    }

    private boolean shouldBePrimeColor(int index, int firstInMovingRange, int lastInMovingRange){
        return (index >= firstInMovingRange && index <= lastInMovingRange) ||
                (index >= lastInMovingRange % ledStrip.getLength() - amountOfMovingLeds &&
                        index <= lastInMovingRange % ledStrip.getLength()) || ((ledStrip.getLength() - index) >= firstInMovingRange && (ledStrip.getLength() - index) <= lastInMovingRange) ||
                ((ledStrip.getLength() - index) >= lastInMovingRange % ledStrip.getLength() - amountOfMovingLeds &&
                        (ledStrip.getLength() - index) <= lastInMovingRange % ledStrip.getLength());
    }

    private void defineColors(Color[] colors, int firstInMovingRange, int lastInMovingRange){
        for (int j = 0; j < ledStrip.getLength(); j++) {
            if (shouldBePrimeColor(firstInMovingRange, lastInMovingRange, j)){
                colors[(ledStrip.getLength() - j) % ledStrip.getLength()] = primeColor;
                colors[j] = primeColor;
            }else {
                colors[(ledStrip.getLength() - j) % ledStrip.getLength()] = backgroundColor;
                colors[j] = backgroundColor;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        Color[] colors = new Color[ledStrip.getLength()];
        for (int i = 0; i < ledStrip.getLength(); i++){
            colors[i] = Color.kBlack;
        }
        ledStrip.setLedsColors(colors);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
