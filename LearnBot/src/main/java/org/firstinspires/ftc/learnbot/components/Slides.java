package org.firstinspires.ftc.learnbot.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.technototes.library.command.Command;
import com.technototes.library.command.CommandScheduler;
import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.logger.Log;
import com.technototes.library.logger.Loggable;
import com.technototes.library.subsystem.Subsystem;
import com.technototes.library.util.PIDFController;

public class Slides {

    @Configurable
    public static class Config {
        //hardware config name
        public static String MotorName = "m";

        // is it reversed?
        public static boolean Reversed = false;

        public static int ZeroPosition = 0;
        public static int LowPosition = 217;
        public static int MiddlePosition = 434;
        public static int UpPosition = 650;
        public static int MaxExtensionTicks = 650;


        public static PIDFCoefficients SlidesPID = new PIDFCoefficients(0.008, 0.0005, 0.00015, 0.0);
        public static double kGlow = 0.06;
        public static double kGhigh = 0.11;

        // How close (in encoder ticks) counts as "at position" for isAtTarget().
        public static double PositionThreshold = 6;
    }


    public static class Commands {

        protected static Component component = null;

        public static Command UP() {
            return Command.create(component::UP);
        }

        public static Command MIDDLE() {
            return Command.create(component::MIDDLE);
        }

        public static Command LOW() {
            return Command.create(component::LOW);
        }

        public static Command ZERO() {
            return Command.create(component::ZERO);
        }
    }

    public enum Position {
        Zero,
        Low,
        Middle,
        Up,
    }

    @Configurable
    public static class Component implements Loggable, Subsystem {


        public Position targetPosition = Position.Zero;

        @Log.Number(name = "Slides Target Ticks")
        public int targetTicks;

        @Log.Number(name = "Slides Current Ticks")
        public double currentTicks;

        @Log.Number(name = "Slides Power")
        public double motorPower;

        private final PIDFController pidfController;

        private final EncodedMotor<DcMotorEx> motor;

        public Component(EncodedMotor<DcMotorEx> slidesMotor) {
            Commands.component = this;
            motor = slidesMotor;
            if (motor != null) {
                motor.setDirection(Config.Reversed ? Direction.REVERSE : Direction.FORWARD);
                motor.brake();
            }
            if (currentTicks < 310) {
                pidfController = new PIDFController(Config.SlidesPID, (target, error) -> Config.kGlow);
            } else  {
                pidfController = new PIDFController(Config.SlidesPID, (target, error) -> Config.kGhigh);
            }
            pidfController.setTarget(Config.ZeroPosition);
            targetTicks = Config.ZeroPosition;
            CommandScheduler.register(this);
        }

        public void UP() {
            setTarget(Position.Up, Config.UpPosition);
        }

        public void MIDDLE() {
            setTarget(Position.Middle, Config.MiddlePosition);
        }

        public void LOW() {
            setTarget(Position.Low, Config.LowPosition);
        }

        public void ZERO() {
            setTarget(Position.Zero, Config.ZeroPosition);
        }

        private void setTarget(Position named, int ticks) {
            targetPosition = named;
            targetTicks = Math.max(0, Math.min(ticks, Config.MaxExtensionTicks));
            pidfController.setTarget(targetTicks);
        }


        @Override
        public void periodic() {
            if (motor != null) {
                currentTicks = motor.getSensorValue();
                motorPower = pidfController.update(currentTicks);
                motor.setPower(motorPower);
            }
        }
    }
}