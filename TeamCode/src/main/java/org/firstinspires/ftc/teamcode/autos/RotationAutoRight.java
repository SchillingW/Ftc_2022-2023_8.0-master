package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBotTesting;
import org.firstinspires.ftc.teamcode.hardware.VisionDevice;

@Autonomous(name="RotationAutoRight", group="PursuitBot")
public class RotationAutoRight extends LinearOpMode {

    public PursuitBotTesting robot;
    public VisionDevice vision;
    public LinearSlide linearSlide;
    ColorSensor sensorColor;
    public int dropOffset = 80;
    public ColorSensor sensor;

    public boolean moveToNext;

    //auto
    @Override
    public void runOpMode() {

        robot = new PursuitBotTesting(telemetry, hardwareMap);
        robot.xDim.cellcorner2botanchorPLACEMENT = 0.125;
        robot.yDim.cellcorner2botanchorPLACEMENT = 2;
        sensor = hardwareMap.colorSensor.get("sensor");
        robot.xDim.cellPLACEMENT = 0;
        robot.yDim.cellPLACEMENT = 4;

        vision = new VisionDevice(telemetry, hardwareMap);
        vision.init();

        linearSlide = new LinearSlide(telemetry, hardwareMap);
        sleep(1000);
        int result = 0;
        while (!isStarted()) {
            int next = vision.perform(1f / 3f);
            if (next != -1) result = next;
            telemetry.addData("current result", result);
            telemetry.update();
        }
        // START MOVEMENT
        waitForStart();
        //hello
        if (opModeIsActive()) linearSlide.closeClaw();
        if (opModeIsActive()) sleep(200);
        linearSlide.goToFull(linearSlide.driveHeight, telemetry, this);

        // CONE GRABBED
        robot.reachPoint(new Pose2d(robot.xDim.toCell(0), robot.yDim.toCell(4), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(4), new Rotation2d()), telemetry, this);

        linearSlide.goToFull(linearSlide.high, telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toPole(2) - 2, robot.yDim.toPole(3), new Rotation2d()), telemetry, this);
        linearSlide.goToFull(linearSlide.high + dropOffset, telemetry, this);
        sleep(200);
        if (opModeIsActive()) linearSlide.openClaw();
        linearSlide.goToFull(linearSlide.stackDriveHeight, telemetry, this);
        Cycle(0);
        Cycle(1);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(result + 3), new Rotation2d()), telemetry, this);
        linearSlide.goToFull(linearSlide.ground, telemetry, this);
    }

    public void GrabConeFromCell(int i)
    {
        linearSlide.goToFull(linearSlide.stacks[i], telemetry, this);
        sleep(500);
        linearSlide.closeClaw();
        sleep(200);
        linearSlide.goToFull(linearSlide.stackDriveHeight, telemetry, this);
    }

    public void Cycle(int i)
    {
        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(5) - 3.35, Rotation2d.fromDegrees(90)), telemetry, this);
        GrabConeFromCell(i);
        linearSlide.goToFull(linearSlide.high, telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toPole(2) - 2, robot.yDim.toPole(3), new Rotation2d()), telemetry, this);
        linearSlide.goToFull(linearSlide.high + dropOffset, telemetry, this);
        sleep(200);
        if (opModeIsActive()) linearSlide.openClaw();
        linearSlide.goToFull(linearSlide.stackDriveHeight, telemetry, this);
    }
}

