package frc.trigon.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class LoggableSubsystemBase extends SubsystemBase implements Loggable {
    @Log(name=".hasDefault")
    public boolean hasDefaultCommand(){
        return getDefaultCommand() != null;
    }

    @Log(name=".default")
    public String getDefaultCommandName(){
        return getDefaultCommand() != null ? getDefaultCommand().getName() : "none";
    }

    @Log(name=".hasCommand")
    public boolean isRunningCommand(){
        return getCurrentCommand() != null;
    }

    @Log(name=".command")
    public String getCurrentCommandName(){
        return getCurrentCommand() != null ? getCurrentCommand().getName() : "none";
    }
}
