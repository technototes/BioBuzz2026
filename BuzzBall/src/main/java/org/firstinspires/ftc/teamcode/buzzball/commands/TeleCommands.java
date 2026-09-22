package org.firstinspires.ftc.teamcode.buzzball.commands;

import com.pedropathing.math.Pose;
import com.technototes.library.command.Command;
import org.firstinspires.ftc.teamcode.buzzball.Robot;

public class TeleCommands {

    public static Command LLRelocCommand(Robot r) {
        return Command.create(new LLRelocCommand(r));
    }

    public static Command TurnTo90(Robot r) {
        return Command.create(() -> r.follower.hold(new Pose(90, 90, Math.toRadians(90)), false)); //pose might need to be current pose?
    }

    public static Command Launch(Robot r) {
        return Command.create(r.launcherSubsystem::Launch);
    }

    public static Command SlidesHigh(Robot r) {
        return Command.create(r.depositSubsystem::slidesToUp);
    }

    public static Command SlidesLow(Robot r) {
        return Command.create(r.depositSubsystem::slidesToLow);
    }

    public static Command SlidesZero(Robot r) {
        return Command.create(r.depositSubsystem::slidesToZero);
    }

    public static Command SetLowBasket(Robot r) {
        return Command.create(r.depositSubsystem::targetToLowBasket);
    }

    public static Command SetHighBasket(Robot r) {
        return Command.create(r.depositSubsystem::targetToHighBasket);
    }

    public static Command ArmDown(Robot r) {
        return Command.create(r.depositSubsystem::armToDown);
    }

    public static Command ArmHoriz(Robot r) {
        return Command.create(r.depositSubsystem::armToHorizDepo);
    }

    public static Command DepositClawOpen(Robot r) {
        return Command.create(r.depositSubsystem::openClaw);
    }

    public static Command DepositClawClose(Robot r) {
        return Command.create(r.depositSubsystem::closeClaw);
    }

    public static Command WristHoriz(Robot r) {
        return Command.create(r.depositSubsystem::wristHoriz);
    }

    public static Command ArmCompensateOn(Robot r) {
        return Command.create(r.depositSubsystem::armCompensationOn);
    }

    public static Command ArmCompensateOff(Robot r) {
        return Command.create(r.depositSubsystem::armCompensationOff);
    }

    public static Command StopLaunch(Robot r) {
        return Command.create(r.launcherSubsystem::Stop);
    }

    public static Command IdleLaunch(Robot r) {
        return Command.create(r.launcherSubsystem::Idle);
    }

    public static Command Intake(Robot r) {
        return Command.create(r.intakeSubsystem::Intake);
    }

    public static Command Feed(Robot r) {
        return Command.create(r.intakeSubsystem::Feed);
    }

    public static Command IntakeStop(Robot r) {
        return Command.create(r.intakeSubsystem::StopIntake);
    }

    public static Command GobbleGulp(Robot r) {
        return Command.create(r.intakeSubsystem::GobbleGulp);
    }

    public static Command IThinkIAteTooMuch(Robot r) {
        return Command.create(r.intakeSubsystem::IThinkIAteTooMuch);
    }

    public static Command Spit(Robot r) {
        return Command.create(r.intakeSubsystem::Spit);
    }

    public static Command EngageBrake(Robot r) {
        return Command.create(r.brakeSubsystem::Engage);
    }

    public static Command DisengageBrake(Robot r) {
        return Command.create(r.brakeSubsystem::Disengage);
    }

    public static Command HoodUp(Robot r) {
        return Command.create(r.aimingSubsystem::testHoodUp);
    }

    public static Command GateUp(Robot r) {
        return Command.create(r.aimingSubsystem::StopBall);
    }

    public static Command GateDown(Robot r) {
        return Command.create(r.aimingSubsystem::GoBall);
    }

    public static Command MotorPowerTest(Robot r) {
        return Command.create(r.testSubsystem::setMotorPowerTest);
    }

    public static Command MotorVelocityTest(Robot r) {
        return Command.create(r.testSubsystem::setMotorVelocityTest);
    }

    public static Command SetRegressionCTeleop(Robot r) {
        return Command.create(r.launcherSubsystem::setRegressionCTeleop);
    }

    public static Command SetRegressionDTeleop(Robot r) {
        return Command.create(r.launcherSubsystem::setRegressionDTeleop);
    }

    public static Command TurretTrackDefault(Robot r) {
        return Command.create(r.turretSubsystem::trackDefaultTarget);
    }

    public static Command TurretDisableTracking(Robot r) {
        return Command.create(() -> {
            r.turretSubsystem.disableTracking();
            r.turretSubsystem.turretToForward();
        });
    }

    public static Command TurretRelocalizeToPosition1(Robot r) {
        return Command.create(r.turretSubsystem::relocalizeToPosition1);
    }

    public static Command TurretRelocalizeToPosition2(Robot r) {
        return Command.create(r.turretSubsystem::relocalizeToPosition2);
    }
}
