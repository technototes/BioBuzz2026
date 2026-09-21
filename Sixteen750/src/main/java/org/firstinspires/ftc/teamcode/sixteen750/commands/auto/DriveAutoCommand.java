package org.firstinspires.ftc.teamcode.sixteen750.commands.auto;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.technototes.library.command.Command;

public class DriveAutoCommand implements Command {

    public Follower follower;
    DrivePowers p;

    public DriveAutoCommand(Follower f, double power) {
        follower = f;
        p = new DrivePowers(power, 0, 0);
    }

    @Override
    public void execute() {
        follower.drivetrain.drive(p, false);
    }
}
