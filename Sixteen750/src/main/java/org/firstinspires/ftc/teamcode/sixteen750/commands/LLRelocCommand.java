package org.firstinspires.ftc.teamcode.sixteen750.commands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.technototes.library.command.Command;
import com.technototes.library.util.Alliance;
import org.firstinspires.ftc.teamcode.sixteen750.Robot;
import org.firstinspires.ftc.teamcode.sixteen750.subsystems.LimelightSubsystem;

@Configurable
public class LLRelocCommand implements Command {

    public Robot robot;
    public LimelightSubsystem ll;
    Pose currentPose;

    public LLRelocCommand(Robot r) {
        robot = r;
        ll = r.limelightSubsystem;
        //pid.setInputBounds(-maxvalue, maxvalue);
    }

    public void execute() {
        if (robot.alliance == Alliance.RED) {
            if (ll.getRPose() != null) {
                robot.follower.setPose(ll.getRPose());
            } else {
                currentPose = robot.follower.pose();
            }
        } else if (robot.alliance == Alliance.BLUE) {
            if (ll.getBPose() != null) {
                robot.follower.setPose(ll.getBPose());
            } else {
                currentPose = robot.follower.pose();
            }
        } else {
            currentPose = robot.follower.pose();
        }
    }
}
