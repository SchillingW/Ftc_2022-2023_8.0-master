package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.hardware.VisionDevice;

@Autonomous(name="AutoTest", group="PursuitBot")
public class AutoTest extends LinearOpMode {

    public PursuitBot robot;
    public VisionDevice vision;
    public ElapsedTime timer = new ElapsedTime();
    public LinearSlide linearSlide;

    public boolean moveToNext;
    public int low = -850; public int med = -1600; public int high = -3050;

    @Override
    public void runOpMode() {

        robot = new PursuitBot(telemetry, hardwareMap);
        robot.xDim.cellcorner2botanchorPLACEMENT = 1;
        robot.yDim.cellcorner2botanchorPLACEMENT = 2;
        robot.xDim.cellPLACEMENT = 0;
        robot.yDim.cellPLACEMENT = 1;

        linearSlide = new LinearSlide(telemetry, hardwareMap);
        moveToNext = false;

        vision = new VisionDevice(telemetry, hardwareMap);
        vision.init();
        int result = 0;
        while (!isStarted()) result = vision.perform(1f / 3f);
        telemetry.addData("result", result);
        telemetry.update();
        waitForStart();

        // START MOVEMENT

        if (opModeIsActive()) linearSlide.closeClaw();
        if (opModeIsActive()) sleep(2000);
        linearSlide.goToFull(linearSlide.low, telemetry, this);

        // CONE GRABBED

        robot.reachPoint(new Pose2d(robot.xDim.toCell(0), robot.yDim.toCell(2), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(2), new Rotation2d()), telemetry, this);
        linearSlide.goToFull(linearSlide.high, telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toPole(1), robot.yDim.toPole(2), new Rotation2d()), telemetry, this);

        // AT DROP CONE LOCATION
        linearSlide.goToFull(linearSlide.med, telemetry, this);
        //if (opModeIsActive()) sleep(1000);
        if (opModeIsActive()) linearSlide.openClaw();
        //if (opModeIsActive()) slide.set(1);


        if (opModeIsActive()) sleep(2000);

        // CONE DROPPED

        robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(2), new Rotation2d()), telemetry, this);
        //robot.reachPoint(new Pose2d(robot.xDim.toCell(0), robot.yDim.toCell(2), new Rotation2d()), telemetry, this);
        linearSlide.goToFull(linearSlide.low, telemetry, this);

        /*robot.reachPoint(new Pose2d(robot.xDim.toCell(0), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCone(1) - 1, robot.yDim.toCone(1), new Rotation2d()), telemetry, this);

        // AT CONE GRAB LOCATION

        linearSlide.goTo(linearSlide.ground, telemetry);
        if (opModeIsActive()) sleep(1000);
        if (opModeIsActive()) linearSlide.closeClaw();
        if (opModeIsActive()) sleep(2000);

        linearSlide.goTo(linearSlide.low, telemetry);;

        // CONE GRABBED hi

        robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        if (opModeIsActive()) linearSlide.goTo(linearSlide.high, telemetry);
        robot.reachPoint(new Pose2d(robot.xDim.toPole(1), robot.yDim.toPole(2), new Rotation2d()), telemetry, this);

        // AT DROP LOCATION

        if (opModeIsActive()) sleep(2000);
        //if (opModeIsActive()) sleep(500);
        //if (opModeIsActive()) slide.set(1);
        if (opModeIsActive()) linearSlide.openClaw();
        if (opModeIsActive()) sleep(2000);*/

        // PARK
        robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(result), new Rotation2d()), telemetry, this);
        linearSlide.goToFull(linearSlide.ground, telemetry, this);
    }
}

