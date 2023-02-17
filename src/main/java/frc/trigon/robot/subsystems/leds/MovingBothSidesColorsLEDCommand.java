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

        for (int i = 0; i < ledStrip.getLength(); i++) {

            if ((i >= firstInMovingRange && i <= lastInMovingRange) ||
                    (i >= lastInMovingRange % ledStrip.getLength() - amountOfMovingLeds &&
                    i <= lastInMovingRange % ledStrip.getLength()) || ((ledStrip.getLength() - i) >= firstInMovingRange && (ledStrip.getLength() - i) <= lastInMovingRange) ||
                    ((ledStrip.getLength() - i) >= lastInMovingRange % ledStrip.getLength() - amountOfMovingLeds &&
                    (ledStrip.getLength() - i) <= lastInMovingRange % ledStrip.getLength())){
                colors[(ledStrip.getLength() - i) % ledStrip.getLength()] = primeColor;
                colors[i] = primeColor;
            }else {
                colors[(ledStrip.getLength() - i) % ledStrip.getLength()] = backgroundColor;
                colors[i] = backgroundColor;
            }
        }
        ledStrip.setLedsColors(colors);

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
