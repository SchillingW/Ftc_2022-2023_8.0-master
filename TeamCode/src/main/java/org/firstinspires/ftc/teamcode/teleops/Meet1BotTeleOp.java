package org.firstinspires.ftc.teamcode.teleops;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.hardware.GamepadSystem;
import com.qualcomm.robotcore.util.ElapsedTime;

// Rohan's teleop to test claw for cones
@TeleOp(name="Meet1BotTeleOp", group="ClawLiftBot")
public class Meet1BotTeleOp extends OpMode {

    PursuitBot robot;

    public double armSpeed = 0.75;
    public int startHeight, reachHeight;
    public int low, med, high;

    public double turnSpeed = 0.55;
    public double linearSpeed = 0.55;

    public boolean moveToNext;
    public boolean manualControl;

    // motor declaration
    public Motor slide;
    public Servo claw;

    // input system reference
    GamepadSystem input;

    // called on program initialization
    @Override
    public void init() {

        robot = new PursuitBot(telemetry, hardwareMap);

        // initialize hardware devicesklm
        //claw = hardwareMap.servo.get("claw"); hi
        claw = hardwareMap.servo.get("claw");
        claw.getController().pwmEnable();

        slide = new Motor(hardwareMap, "slide");
        slide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        reachHeight = 0;

        input = new GamepadSystem(this);

        moveToNext = false;
        manualControl = false;
    }

    // called repeatedly during program
    @Override
    public void loop() {

        telemetry.addData("Drive Speed", linearSpeed);
        telemetry.addData("Slide Pos", slide.encoder.getPosition());
        telemetry.addData("Desired Pos", reachHeight);
        telemetry.addData("Absolute Value", Math.abs(slide.encoder.getPosition() - reachHeight));
        telemetry.addData("Slide Joystick Input", input.gamepad2.getRightY());

        // NORMAL
        robot.drive.driveRobotCentric(
                input.gamepad1.getLeftY() * Math.abs(input.gamepad1.getLeftY()) * linearSpeed,
                input.gamepad1.getLeftX() * Math.abs(input.gamepad1.getLeftX()) * linearSpeed,
                input.gamepad1.getRightX() * Math.abs(input.gamepad1.getRightX()) * turnSpeed);

        /*
        // GYRO COMP
        robot.odometry.update();
        robot.drive.driveFieldCentric(
                input.gamepad1.getLeftY() * Math.abs(input.gamepad1.getLeftY()) * linearSpeed,
                input.gamepad1.getLeftX() * Math.abs(input.gamepad1.getLeftX()) * linearSpeed,
                input.gamepad1.getRightX() * Math.abs(input.gamepad1.getRightX()) * turnSpeed,
                robot.odometry.getPose().getHeading() / 2 / Math.PI * 360);
         */

        telemetry.addData("heading", robot.odometry.getPose().getHeading());

        //if(input.gamepad2.getRightY() <= -0.01) slide.set(input.gamepad2.getRightY() * armSpeed - 0.1);
        if(moveToNext && !manualControl)
        {
            if(Math.abs(slide.encoder.getPosition() - reachHeight) < 20)
            {
                moveToNext = false;
                return;
            }
            slide.set(Math.signum(reachHeight - slide.encoder.getPosition()));
        }
        else if(manualControl)
        {
            reachHeight = slide.encoder.getPosition();
            slide.set(input.gamepad2.getRightY() * armSpeed);
        }
        else slide.set(-0.1);
        //full close=1 hi hi  hi

        if(input.gamepad1.getButton(GamepadKeys.Button.X))
        {
            linearSpeed = 0.75;
            turnSpeed = 0.75;
        }

        if(input.gamepad1.getButton(GamepadKeys.Button.Y))
        {
            linearSpeed = 0.55;
            turnSpeed = 0.55;
        }
//
        if(input.gamepad1.getButton(GamepadKeys.Button.B))
        {
            linearSpeed = 0.35;
            turnSpeed = 0.35;
        }
//
        if (input.gamepad2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            claw.setPosition(0);
            telemetry.addData("servo", "close");
            telemetry.update();
        }
        //full open=0
        if (input.gamepad2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            claw.setPosition(0.5);
            telemetry.addData("servo", "open");
            telemetry.update();
        }

        if(input.gamepad2.getButton(GamepadKeys.Button.X))
        {
            moveToNext = true;
            reachHeight = -850;
        }

        if(input.gamepad2.getButton(GamepadKeys.Button.Y))
        {
            moveToNext = true;
            reachHeight = -1600;
        }

        if(input.gamepad2.getButton(GamepadKeys.Button.B))
        {
            moveToNext = true;
            reachHeight = -3050;
        }

        if(input.gamepad2.getButton(GamepadKeys.Button.A))
        {
            moveToNext = true;
            reachHeight = 0;
        }

        manualControl = (input.gamepad2.getRightY() != 0);
    }
}
