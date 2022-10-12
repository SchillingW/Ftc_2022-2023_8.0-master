package org.firstinspires.ftc.teamcode.teleops;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.hardware.GamepadSystem;

// Rohan's teleop to test claw for cones
@TeleOp(name="Meet1BotTeleOp", group="ClawLiftBot")
public class Meet1BotTeleOp extends OpMode {

    PursuitBot robot;

    public double armSpeed = 0.75;

    public double turnSpeed = 0.75;
    public double linearSpeed = 0.75;

    // motor declaration
    public Motor slide;
    public ServoEx claw;

    // input system reference
    GamepadSystem input;

    // called on program initialization
    @Override
    public void init() {

        robot = new PursuitBot(telemetry, hardwareMap);

        // initialize hardware devicesklm
        claw = new SimpleServo(hardwareMap, "claw", 0, 180);
        slide = new Motor(hardwareMap, "slide");

        input = new GamepadSystem(this);
    }

    // called repeatedly during program
    @Override
    public void loop() {

        robot.drive.driveRobotCentric(
                input.gamepad1.getLeftY() * linearSpeed,
                input.gamepad1.getLeftX() * linearSpeed,
                input.gamepad1.getRightX() * turnSpeed);

        slide.set(-input.gamepad2.getRightY() * armSpeed);
        //full close=1

        if (input.gamepad2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            claw.setPosition(0);
            telemetry.addData("servo", "close");
            telemetry.update();
        }
        //full open=0
        if (input.gamepad2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            claw.setPosition(0.35);
            telemetry.addData("servo", "open");
            telemetry.update();
        }

    }
}