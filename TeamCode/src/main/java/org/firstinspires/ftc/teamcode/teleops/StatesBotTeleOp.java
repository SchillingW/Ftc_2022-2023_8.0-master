package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBotTesting;
import org.firstinspires.ftc.teamcode.hardware.GamepadSystem;

@TeleOp(name="Meet1BotTeleOp", group="ClawLiftBot")
public class StatesBotTeleOp extends OpMode {

    PursuitBotTesting robot;
    LinearSlide linearSlide;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    RevBlinkinLedDriver lights;


    public double turnSpeed = 0.75;
    public double linearSpeed = 0.75;

    public int heightIndex = 0;
    public boolean joystickControl = false;

    public boolean lastUp;
    public boolean lastDown;
    public ColorSensor sensor;

    public double baseHeading;



    // input system reference
    GamepadSystem input;
    final double SCALE_FACTOR = 255;
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // called on program initialization
    @Override
    public void init() {

        robot = new PursuitBotTesting(telemetry, hardwareMap);
        input = new GamepadSystem(this);
        linearSlide = new LinearSlide(telemetry, hardwareMap);
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        sensor = hardwareMap.colorSensor.get("sensor");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor");

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
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);

        // GYRO COMP

        /*robot.odometry.update();
        robot.drive.driveFieldCentric(
                input.gamepad1.getLeftY() * Math.abs(input.gamepad1.getLeftY()) * linearSpeed,
                input.gamepad1.getLeftX() * Math.abs(input.gamepad1.getLeftX()) * linearSpeed,
                input.gamepad1.getRightX() * Math.abs(input.gamepad1.getRightX()) * turnSpeed,
                robot.odometry.getPose().getHeading() / 2 / Math.PI * 360 - baseHeading);*/



        // reorient forward to current direction
        if (gamepad1.right_bumper) baseHeading = robot.odometry.getPose().getHeading() / 2 / Math.PI * 360;


        telemetry.addData("heading", robot.odometry.getPose().getHeading() / 2 / Math.PI * 360);
        telemetry.addData("base heading", baseHeading);
        /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
        telemetry.addData("Alpha", sensorColor.alpha());
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());*/

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
            linearSlide.closeClaw();
            telemetry.addData("servo", "close");
            telemetry.update();
        }
        //full open=0
        if (input.gamepad2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            linearSlide.openClaw();
            telemetry.addData("servo", "open");
            telemetry.update();
        }

        if (sensorDistance.getDistance(DistanceUnit.CM) <= 2.5) {
            gamepad2.rumble(500);
        }

        boolean thisUp = input.gamepad2.getButton(GamepadKeys.Button.DPAD_UP);
        boolean thisDown = input.gamepad2.getButton(GamepadKeys.Button.DPAD_DOWN);
        if (thisUp && !lastUp) {heightIndex++; joystickControl = false;}
        if (thisDown & !lastDown) {heightIndex--; joystickControl = false;}
        heightIndex = Math.max(0, Math.min(linearSlide.slidePositions.length - 1, heightIndex));
        telemetry.addData("heightIndex", heightIndex);
        lastUp = thisUp;
        lastDown = thisDown;

        if (input.gamepad2.getRightY() != 0 || joystickControl)
        {
            linearSlide.moveByJoystick(input.gamepad2.getRightY());
            joystickControl = true;
        }
        else
        {
            linearSlide.goTo(linearSlide.slidePositions[heightIndex], telemetry);
        }
    }
}
