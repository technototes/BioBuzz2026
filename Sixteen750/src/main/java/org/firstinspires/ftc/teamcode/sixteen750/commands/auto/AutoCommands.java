package org.firstinspires.ftc.teamcode.sixteen750.commands.auto;

import com.technototes.library.command.Command;
import com.technototes.library.command.ParallelCommandGroup;
import com.technototes.library.command.ParallelRaceGroup;
import com.technototes.library.command.SequentialCommandGroup;
import com.technototes.library.command.WaitCommand;
import org.firstinspires.ftc.teamcode.sixteen750.Robot;
import org.firstinspires.ftc.teamcode.sixteen750.subsystems.IntakeSubsystem;

public class AutoCommands {

    public static SequentialCommandGroup Fire(Robot r) {
        return new SequentialCommandGroup(r.intakeSubsystem::Feed, new WaitCommand(.92));
    }

    public static SequentialCommandGroup FireHalf(Robot r) {
        return new SequentialCommandGroup(
            r.intakeSubsystem::Feed,
            new WaitCommand(.3),
            r.intakeSubsystem::Hold
        );
    }

    abstract static class AutoTurret implements Command {

        @Override
        public boolean isFinished() {
            return false;
        }

        public static Command AutoTrack(Robot r) {
            return Command.create(r.turretsubsystem::setTurretTarget);
        }
    }

    abstract static class AutoLaunch implements Command {

        @Override
        public boolean isFinished() {
            return false;
        }

        public static Command AutoVelo(Robot r) {
            return Command.create(r.turretsubsystem::Launch);
        }
    }

    static class IntakeFull implements Command {

        IntakeSubsystem intake;

        public IntakeFull(Robot r) {
            this.intake = r.intakeSubsystem;
        }

        @Override
        public boolean isFinished() {
            return intake.IntakeFull(); // This should return if the intake is full
            // return 'intake.IsRobotFull()' or something
        }

        @Override
        public void execute() {}
    }

    public static Command WaitForFullIntake(Robot r, double timeout) {
        return new ParallelRaceGroup(new IntakeFull(r), new WaitCommand(timeout));
    }
}
