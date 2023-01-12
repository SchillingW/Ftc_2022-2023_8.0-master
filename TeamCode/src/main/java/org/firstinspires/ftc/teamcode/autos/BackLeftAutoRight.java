package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBotTesting;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBotTesting2;
import org.firstinspires.ftc.teamcode.hardware.VisionDevice;

@Autonomous(name="BaccLeftt", group="PursuitBot")
public class BackLeftAutoRight extends LinearOpMode {

    public PursuitBotTesting2 robot;
    public VisionDevice vision;
    public LinearSlide linearSlide;
    ColorSensor sensorColor;
    public int dropOffset = 40;
    public ColorSensor sensor;

    public boolean moveToNext;

    //auto
    @Override
    public void runOpMode() {

        robot = new PursuitBotTesting2(telemetry, hardwareMap);
        robot.xDim.cellcorner2botanchorPLACEMENT = 0.375;
        robot.yDim.cellcorner2botanchorPLACEMENT = 9;
        sensor = hardwareMap.colorSensor.get("sensor");
        robot.xDim.cellPLACEMENT = 4;
        robot.yDim.cellPLACEMENT = -1;

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
        if (opModeIsActive()) linearSlide.closeClaw();
        sleep(200);

        // CONE GRABBED
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(3), robot.yDim.toCell(-1), new Rotation2d()), telemetry, this, linearSlide, linearSlide.driveHeight, false);
        robot.reachPointSlide(new Pose2d(robot.xDim.toPole(3) + 2, robot.yDim.toPole(-4) + 1, new Rotation2d()), telemetry, this, linearSlide, linearSlide.high, true);
        if (opModeIsActive()) linearSlide.openClaw();

        Cycle(0);
        Cycle(1);

        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(3), robot.yDim.toCell(-3), new Rotation2d()), telemetry, this, linearSlide, linearSlide.low, false);
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(result + 3), robot.yDim.toCell(-3), new Rotation2d()), telemetry, this, linearSlide, linearSlide.ground, true);
    }

    public void Cycle(int i)
    {
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(3), robot.yDim.toCell(-3), new Rotation2d()), telemetry, this, linearSlide, linearSlide.low, false);
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(5) + 1.5, robot.yDim.toCell(-3), new Rotation2d()), telemetry, this, linearSlide, linearSlide.stacks[i] - 75, true);
        linearSlide.closeClaw();
        sleep(200);
        //linearSlide.goToFull(linearSlide.stackDriveHeight, telemetry, this);

        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(3), robot.yDim.toCell(-3), new Rotation2d()), telemetry, this,  linearSlide, linearSlide.high, false);
        robot.reachPointSlide(new Pose2d(robot.xDim.toPole(3) + 2, robot.yDim.toPole(-4) + 1, new Rotation2d()), telemetry, this, linearSlide, linearSlide.high, true);
        if (opModeIsActive()) linearSlide.openClaw();
        sleep(200);
    }
}

