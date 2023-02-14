package org.firstinspires.ftc.teamcode.oldproj.autos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.oldproj.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.oldproj.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.oldproj.botconfigs.PursuitBotTesting;
import org.firstinspires.ftc.teamcode.oldproj.hardware.VisionDevice;

@Autonomous(name="ROTRIGHT", group="PursuitBot")
public class ROTRIGHT extends LinearOpMode {

    public double poleCellDiff;
    public PursuitBotTesting robot;
    public VisionDevice vision;
    public LinearSlide linearSlide;
    ColorRangeSensor sensorColor;
    public int dropOffset = 225;
    public ColorSensor sensor;

    public boolean moveToNext;

    public Pose2d high;
    public Pose2d stack;
    public Pose2d midCell;
    public Pose2d midPoint;
    public Pose2d midPoint2;
    public Pose2d midCell2;
    public Pose2d beforePole;
    public Pose2d beforeRot;
    public Pose2d med;
    public Pose2d beforeStack;

    //auto
    @Override
    public void runOpMode() {
        robot = new PursuitBotTesting(telemetry, hardwareMap);
        robot.xDim.cellcorner2botanchorPLACEMENT = 0.125;
        robot.yDim.cellcorner2botanchorPLACEMENT = 2;
        sensor = hardwareMap.colorSensor.get("sensor");
        robot.xDim.cellPLACEMENT = 0;
        robot.yDim.cellPLACEMENT = 4;

        poleCellDiff = robot.xDim.toPole(1) - robot.xDim.toCell(1) - 1.5;
        high = new Pose2d(robot.xDim.toCell(2) + 2.75, robot.yDim.toPole(3) - 1, new Rotation2d());
        stack = new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(5) - 1.35, Rotation2d.fromDegrees(90));
        med = new Pose2d(robot.xDim.toCell(2) - poleCellDiff + 1.5, robot.yDim.toPole(3) - 2, Rotation2d.fromDegrees(185));
        midPoint = new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(4), new Rotation2d());
        midCell = new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(4), Rotation2d.fromDegrees(90));
        midPoint2 = new Pose2d(robot.xDim.toCell(2), robot.yDim.toPole(3), new Rotation2d());
        midCell2 = new Pose2d(robot.xDim.toCell(2), robot.yDim.toPole(3), Rotation2d.fromDegrees(90));
        beforePole = new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(4), Rotation2d.fromDegrees(180));
        beforeRot = new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(4), Rotation2d.fromDegrees(90));
        beforeStack = new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(4), Rotation2d.fromDegrees(95));

        vision = new VisionDevice(telemetry, hardwareMap);
        vision.init();

        linearSlide = new LinearSlide(telemetry, hardwareMap);
        linearSlide.closeClaw();

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

        // CONE GRABBED
        //robot.reachPointSlide(new Pose2d(robot.xDim.toCell(0), robot.yDim.toCell(4), new Rotation2d()), telemetry, this, linearSlide, linearSlide.driveHeight, false);
        robot.setConstants(0.4, 0.3, 16, 4);
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(0), robot.yDim.toCell(4), new Rotation2d()), telemetry, this, linearSlide, linearSlide.driveHeight, false);
        robot.setConstants(0.6, 0.6, 16, 4);
        robot.reachPointSlideNextPoint(new Pose2d(robot.xDim.toCell(1.8), robot.yDim.toCell(4) - 2, new Rotation2d()), high, telemetry, this, linearSlide, linearSlide.med, false);
        robot.setConstants(0.6, 0.25, 16, 2);
        robot.reachPointSlide(high, telemetry, this, linearSlide, linearSlide.high, false);
        robot.drive.stop();
        linearSlide.goToFull(linearSlide.high + dropOffset, telemetry, this);
        if (opModeIsActive()) linearSlide.openClaw();
        robot.reachPointSlide(midPoint, telemetry, this, linearSlide, linearSlide.low, false);
        robot.reachPointSlide(beforeRot, telemetry, this, linearSlide, linearSlide.low, false);


        Cycle(0, false, result);
        Cycle(1, false, result);
        Cycle(2, false, result);
        Cycle(3, true, result);
    }

    public void Cycle(int i, boolean lastCycle, int result)
    {
        Pose2d newMed = new Pose2d(med.getX() + 0.75 * (i + 1), med.getY(), med.getRotation());

        robot.setConstants(0.4, 0.19, 8, 1);
        robot.reachPointSlide(stack, telemetry, this, linearSlide, linearSlide.stacks[i], false);
        robot.drive.stop();
        linearSlide.closeClaw();
        sleep(150);
        linearSlide.goTo(linearSlide.low, telemetry);
        sleep(100);
        robot.setConstants(0.8, 0.25, 10, 1);
        robot.reachPointSlide(newMed, telemetry, this, linearSlide, linearSlide.med, false);
        robot.drive.stop();
        linearSlide.goToFull(linearSlide.med + dropOffset, telemetry, this);
        if (opModeIsActive()) linearSlide.openClaw();
        robot.setConstants(0.8, 0.4, 10, 4);
        robot.reachPointSlideNextPoint(beforeStack, stack, telemetry, this, linearSlide, linearSlide.low, false);
        if(!lastCycle) return;

        robot.setConstants(0.8, 0.19, 12, 6);

        if(result != 2) {
            robot.reachPointSlide(new Pose2d(beforePole.getX(), robot.yDim.toCell(result + 3), beforePole.getRotation()), telemetry, this, linearSlide, linearSlide.stacks[i + 1], false);
            robot.drive.stop();
        }
        else
        {
            robot.reachPointSlide(stack, telemetry, this, linearSlide, linearSlide.stacks[i + 1], false);
            robot.drive.stop();
        }
    }
}

