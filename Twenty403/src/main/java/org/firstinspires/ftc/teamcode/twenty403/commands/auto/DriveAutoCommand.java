package org.firstinspires.ftc.teamcode.twenty403.commands.auto;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.technototes.library.command.Command;

public class DriveAutoCommand implements Command {

    public Follower follower;
    DrivePowers powers;

    public DriveAutoCommand(Follower f, double fwd, double strafe) {
        follower = f;
        powers = new DrivePowers(fwd, strafe, 0);
    }

    @Override
    public void execute() {
        follower.manual(powers);
    }
}
