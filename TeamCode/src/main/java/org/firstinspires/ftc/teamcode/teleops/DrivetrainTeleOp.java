package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBotTesting;
import org.firstinspires.ftc.teamcode.hardware.GamepadSystem;

@TeleOp(name="DrivetrainTeleOp", group="ClawLiftBot")
public class DrivetrainTeleOp extends OpMode {

    PursuitBotTesting robot;

    public double turnSpeed = 1;
    public double linearSpeed = 1;



    // input system reference
    GamepadSystem input;

    // called on program initialization
    @Override
    public void init() {

        robot = new PursuitBotTesting(telemetry, hardwareMap);
        input = new GamepadSystem(this);
    }

    // called repeatedly during program
    @Override
    public void loop() {

        telemetry.addData("Drive Speed", linearSpeed);

        // GYRO COMP
        robot.drive.driveFieldCentric(
                input.gamepad1.getLeftY() * Math.abs(input.gamepad1.getLeftY()) * linearSpeed,
                input.gamepad1.getLeftX() * Math.abs(input.gamepad1.getLeftX()) * linearSpeed,
                input.gamepad1.getRightX() * Math.abs(input.gamepad1.getRightX()) * turnSpeed,
                robot.odometry.getPose().getHeading() / 2 / Math.PI * 360);


        telemetry.addData("heading", robot.odometry.getPose().getHeading());



    }
}
