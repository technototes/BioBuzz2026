package org.firstinspires.ftc.teamcode.buzzball.commands;

import com.bylazar.configurables.annotations.Configurable;
import com.technototes.library.command.Command;
import org.firstinspires.ftc.teamcode.buzzball.Robot;

@Configurable
public class AltAutoVelocity implements Command {

    public Robot robot;

    public AltAutoVelocity(Robot r) {
        robot = r;
    }

    @Override
    public boolean isFinished() {
        //return !robot.follower.isBusy();
        return false;
    }

    @Override
    public void execute() {
        robot.launcherSubsystem.Launch();
    }

    //    @Override
    //    public void end(boolean s) {
    //        robot.follower.drivetrain.breakFollowing();
    //    }
}
