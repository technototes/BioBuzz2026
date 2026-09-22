package org.firstinspires.ftc.teamcode.buzzball.commands.driving;

import com.technototes.library.command.Command;
import org.firstinspires.ftc.teamcode.buzzball.Robot;

public class RestartTeleop implements Command {

    public Robot robot;

    public RestartTeleop(Robot r) {
        robot = r;
    }

    @Override
    public boolean isFinished() {
        return !robot.follower.isBusy();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean s) {
        robot.follower.drivetrain.stop();
    }
}
