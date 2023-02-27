package frc.trigon.robot.subsystems.leds;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

public class MasterLed extends SubsystemBase {
    private static final MasterLed INSTANCE = new MasterLed();
    public static final ArrayList<LedStrip> LED_STRIPS = new ArrayList<>();

    public static MasterLed getInstance() {
        return INSTANCE;
    }

    private MasterLed() {
    }
}

