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

import org.firstinspires.ftc.teamcode.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.hardware.GamepadSystem;
import com.qualcomm.robotcore.util.ElapsedTime;

// Rohan's teleop to test claw for cones
@TeleOp(name="Meet1BotTeleOp", group="ClawLiftBot")
public class Meet1BotTeleOp extends OpMode {

    PursuitBot robot;
    LinearSlide slide;

    public double armSpeed = 0.75;

    public double turnSpeed = 0.55;
    public double linearSpeed = 0.55;

    // input system reference
    GamepadSystem input;

    // called on program initialization
    @Override
    public void init() {

        robot = new PursuitBot(telemetry, hardwareMap);
        input = new GamepadSystem(this);
    }

    // called repeatedly during program
    @Override
    public void loop() {

        telemetry.addData("Drive Speed", linearSpeed);

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
            slide.closeClaw();
            telemetry.addData("servo", "close");
            telemetry.update();
        }
        //full open=0
        if (input.gamepad2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            slide.openClaw();
            telemetry.addData("servo", "open");
            telemetry.update();
        }

        if(input.gamepad2.getButton(GamepadKeys.Button.A)) slide.goTo(slide.ground, telemetry);
        if(input.gamepad2.getButton(GamepadKeys.Button.X)) slide.goTo(slide.low, telemetry);
        if(input.gamepad2.getButton(GamepadKeys.Button.Y)) slide.goTo(slide.med, telemetry);
        if(input.gamepad2.getButton(GamepadKeys.Button.B)) slide.goTo(slide.high, telemetry);
        if(input.gamepad2.getRightY() != 0) slide.moveByJoystick(input.gamepad2.getRightY());
    }
}
