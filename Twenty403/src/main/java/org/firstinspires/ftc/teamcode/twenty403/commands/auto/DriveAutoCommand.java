package org.firstinspires.ftc.teamcode.twenty403.commands.auto;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.technototes.library.command.Command;
import org.firstinspires.ftc.teamcode.twenty403.subsystems.DrivebaseSubsystem;

public class DriveAutoCommand implements Command {

    public DrivebaseSubsystem drivbase;
    double forward = 0;
    double strafe = 0;

    public DriveAutoCommand(DrivebaseSubsystem db, double fwd, double strafe) {
        drivbase = db;
    }

    @Override
    public void execute() {
        drivbase.setControllerInputs(forward, strafe, 0);
    }
}
