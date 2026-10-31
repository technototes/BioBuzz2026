package org.firstinspires.ftc.teamcode.twenty403.commands.auto;

import com.technototes.library.command.Command;
import com.technototes.library.command.SequentialCommandGroup;
import com.technototes.library.command.WaitCommand;
import org.firstinspires.ftc.teamcode.twenty403.Robot;

public class AutoCommands {

    public static SequentialCommandGroup Fire(Robot r) {
        return new SequentialCommandGroup(r.intakeSubsystem::Intake, new WaitCommand(1));
    }

    abstract static class AutoTurret implements Command {

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    abstract static class AutoLaunch implements Command {

        @Override
        public boolean isFinished() {
            return false;
        }

        public static Command AutoVelo(Robot r) {
            return Command.create(r.launcherSubsystem::Launch);
        }
    }
}
