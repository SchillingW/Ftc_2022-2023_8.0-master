package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.botconfigs.LinearSlide;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.hardware.GamepadSystem;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(name = "RoadRunner TeleOp", group = "advanced")
public class RR_Tele extends LinearOpMode {

    //INCORPORATE LINEAR SLIDE AND CLAW FUNC
    LinearSlide linearSlide;

    public int heightIndex = 0;
    public boolean joystickControl = false;

    public boolean lastUp;
    public boolean lastDown;

    GamepadSystem gamepadSystemInput;

    @Override
    public void runOpMode() throws InterruptedException {

        //INCORPORATE LINEAR SLIDE AND CLAW FUNC
        gamepadSystemInput = new GamepadSystem(this);
        linearSlide = new LinearSlide(telemetry, hardwareMap);

        // Initialize SampleMecanumDrive
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        //drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            //Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            //Vector2d input = new Vector2d(
             //       -gamepadSystemInput.gamepad1.getLeftY(),
             //       -gamepadSystemInput.gamepad1.getLeftX()
            //).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            //drive.setWeightedDrivePower(
                    //new Pose2d(
                    //        input.getX(),
                   //         input.getY(),
                    //        -gamepadSystemInput.gamepad1.getRightX()
                    //)
            //);

            // Update everything. Odometry. Etc.
            //drive.update();

            // Print pose to telemetry
            //telemetry.addData("x", poseEstimate.getX());
            //telemetry.addData("y", poseEstimate.getY());
            //telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            //INCORPORATE LINEAR SLIDE AND CLAW FUNCTIONALITY
            if (gamepadSystemInput.gamepad2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                linearSlide.closeClaw();
                telemetry.addData("servo", "close");
                telemetry.update();
            }
            //full open=0
            if (gamepadSystemInput.gamepad2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                linearSlide.openClaw();
                telemetry.addData("servo", "open");
                telemetry.update();
            }

            boolean thisUp = gamepadSystemInput.gamepad2.getButton(GamepadKeys.Button.DPAD_UP);
            boolean thisDown = gamepadSystemInput.gamepad2.getButton(GamepadKeys.Button.DPAD_DOWN);
            if (thisUp && !lastUp) {heightIndex++; joystickControl = false;}
            if (thisDown & !lastDown) {heightIndex--; joystickControl = false;}
            heightIndex = Math.max(0, Math.min(linearSlide.slidePositions.length - 1, heightIndex));
            telemetry.addData("heightIndex", heightIndex);
            lastUp = thisUp;
            lastDown = thisDown;

            if (gamepadSystemInput.gamepad2.getRightY() != 0 || joystickControl)
            {
                linearSlide.moveByJoystick(gamepadSystemInput.gamepad2.getRightY());
                joystickControl = true;
            }
            else
            {
                linearSlide.goTo(linearSlide.slidePositions[heightIndex], telemetry);
            }
        }
    }
}
