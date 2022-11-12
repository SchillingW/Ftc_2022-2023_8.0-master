package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.hardware.GamepadSystem;

@TeleOp(name="DrivetrainTeleOp", group="ClawLiftBot")
public class DrivetrainTeleOp extends OpMode {

    PursuitBot robot;

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


        telemetry.addData("heading", robot.odometry.getPose().getHeading());



    }
}
