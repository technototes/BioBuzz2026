package org.firstinspires.ftc.buzzball.opmodes.auto;

import com.technototes.library.command.Command;
import org.firstinspires.ftc.buzzball.subsystems.SafetySubsystem;

public class SafetyStopCommand implements Command {

    private SafetySubsystem subsystem;

    @Override
    public void execute() {
        subsystem.stopMonitoring();
    }

    public SafetyStopCommand(SafetySubsystem d) {
        subsystem = d;
        addRequirements(d);
    }
}
