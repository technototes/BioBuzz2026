package org.firstinspires.ftc.sixteen750.commands.auto;

import com.technototes.library.command.Command;
import com.technototes.library.command.ParallelCommandGroup;
import com.technototes.library.command.ParallelRaceGroup;
import com.technototes.library.command.SequentialCommandGroup;
import com.technototes.library.command.WaitCommand;
import org.firstinspires.ftc.sixteen750.Robot;
import org.firstinspires.ftc.sixteen750.commands.TeleCommands;
import org.firstinspires.ftc.sixteen750.subsystems.IntakeSubsystem;

public class AutoCommands {

    abstract static class WaitForArtifacts implements Command {

        @Override
        public boolean isFinished() {
            return IntakeSubsystem.robotFull;
        }

        public static Command Intake(Robot r) {
            return Command.create(r.intakeSubsystem::Intake);
        }
    }

    public Command AutoIntake(Robot r) {
        return new ParallelRaceGroup(new WaitCommand(10), WaitForArtifacts.Intake(r));
    }
}
