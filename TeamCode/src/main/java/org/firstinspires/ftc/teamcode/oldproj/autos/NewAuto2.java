package org.firstinspires.ftc.teamcode.oldproj.autos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.oldproj.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.oldproj.botconfigs.TrajectoryBot;

import java.util.ArrayList;

@Autonomous(name="NewAuto2", group="PursuitBot")
public class NewAuto2 extends LinearOpMode {

    public PursuitBot robot;

    //public VisionDevice vision;
    //public LinearSlide linearSlide;
    public int dropOffset = 80;

    public double maxVelocity = 0.5;
    public double maxAcceleration = 0.5;

    public boolean moveToNext;

    //auto
    @Override
    public void runOpMode() {

        robot = new PursuitBot(telemetry, hardwareMap);
        robot.xDim.cellcorner2botanchorPLACEMENT = 1.5;
        robot.yDim.cellcorner2botanchorPLACEMENT = 2.5;
        robot.xDim.cellPLACEMENT = 0;
        robot.yDim.cellPLACEMENT = 1;


        //linearSlide = new LinearSlide(telemetry, hardwareMap);
        sleep(1000);

        waitForStart();

        Trajectory t = createTraj(
                new Pose2d(robot.inchesToMeters(robot.xDim.toCell(2)), robot.inchesToMeters(robot.yDim.toCell(0)), new Rotation2d(Math.toRadians(90))),
                new Translation2d[]{new Translation2d(robot.inchesToMeters(robot.xDim.toCell(2) / 4), robot.inchesToMeters(robot.yDim.toCell(0))),
                        new Translation2d(robot.inchesToMeters(robot.xDim.toCell(2) / 4), robot.inchesToMeters(robot.yDim.toCell(0)))}, false);

        double inc = 1.5;
        double seconds = t.getTotalTimeSeconds();
        ElapsedTime timer = new ElapsedTime();

        while(timer.seconds() < seconds)
        {
            Trajectory.State goal = t.sample(timer.seconds() + inc);
            Pose2d pose = goal.poseMeters;
            robot.moveTowards(false, pose, telemetry);
        }
    }

    public Trajectory createTraj(Pose2d pose, Translation2d[] interiorPoints, boolean reversed) {
        ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
        for (int i = 0; i < interiorPoints.length; i++) {
            interiorWaypoints.add(interiorPoints[i]);
        }
        TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);
        config.setReversed(reversed);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                robot.odometry.getPose(), interiorWaypoints, pose, config);
        return trajectory;
    }
}