package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.purepursuit.waypoints.PointTurnWaypoint;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.tools.javac.util.Convert;

import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import java.util.ArrayList;

// pure pursuit algorithm demo
//hi
@TeleOp(name="TrajectoryDemo", group="PursuitBot")
public class TrajectoryDemo extends LinearOpMode {

    // robot reference
    public PursuitBot robot;

    // robot poses which form recorded path
    public ArrayList<Pose2d> recording;
    public ArrayList<Translation2d> returnHomeInteriorWaypoints = new ArrayList<Translation2d>();
    public ArrayList<Translation2d> followPathInteriorWaypoints = new ArrayList<Translation2d>();

    // movement parameters
    public double movementSpeed = 0.5;
    public double turnSpeed = 0.5;
    public double followRadius = 5;
    public double positionBuffer = 1;
    public double rotationBuffer = Math.toRadians(15);
    public double maxVelocity = 0.5;
    public double maxAcceleration = 0.5;

    public boolean isAtHome = false;
    public boolean isDoneCorrectingRotation = false;
    public boolean positiveStartingX;
    public boolean positiveStartingY;

    @Override
    public void runOpMode() {

        // get reference to robot
        robot = new PursuitBot(telemetry, hardwareMap);

        // wait for user to start program
        waitForStart();

        // keep looping while program is running
        while (opModeIsActive()) {

            // loop through demo states
            RecordPath();
            ReturnHome();
            FollowPath();
            ReturnHome();
        }
    }

    // behaviour for robot driving and user path recording
    public void RecordPath() {

        // check that program is running
        if (opModeIsActive()) {

            // debug
            DebugPartial("record path");

            // reset recorded poses
            recording = new ArrayList<>();
            boolean recordInputLast = gamepad1.b;

            // keep looping while program is running and a is not pressed
            while (opModeIsActive() && (!gamepad1.a || recording.size() < 1)) {

                // debug
                DebugFull("record path");

                // update odometry system
                robot.odometry.update();

                // drive based on controller input
                robot.drive.driveRobotCentric(
                        -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

                // add current pose to recording if b pressed
                boolean recordInputNew = gamepad1.b;
                if (recordInputNew && !recordInputLast) recording.add(robot.odometry.getPose());
                recordInputLast = recordInputNew;
            }

            // stop drive train
            robot.drive.stop();
        }
    }

    public Translation2d ConvertToTranslation2d(Pose2d pose)
    {
        double xValue = pose.getX();
        double yValue = pose.getY();

        return new Translation2d(xValue, yValue);
    }

    // follows recorded path with pure pursuit
    public void FollowPath() {

        // check that program is running
        if (opModeIsActive()) {

            // debug
            DebugPartial("follow path");

            // create start and end waypoints from current pose to last pose in recording
            Pose2d start = robot.odometry.getPose();
            Pose2d end = recording.get(recording.size() - 1);

            for (int i = 1; i < recording.size() - 1; i++)
            {
                double xPos = recording.get(i).getX();
                double yPos = recording.get(i).getY();
                Translation2d translation = new Translation2d(xPos, yPos);
                followPathInteriorWaypoints.add(i - 1, translation);
            }

            positiveStartingY = (start.getY() > 0);
            Translation2d last = followPathInteriorWaypoints.get(followPathInteriorWaypoints.size() - 1);
            Pose2d waypointPose2d = new Pose2d(last.getX(), last.getY(), new Rotation2d());
            GeneralWaypoint finalWaypoint = new GeneralWaypoint(waypointPose2d, movementSpeed,
                    turnSpeed, followRadius);

            TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);
            config.setReversed(DetectReverse(positiveStartingY, finalWaypoint));

            Trajectory followPathTrajectory = TrajectoryGenerator.generateTrajectory(
                    start, followPathInteriorWaypoints, end, config);
            double seconds = followPathTrajectory.getTotalTimeSeconds();

            Waypoint[] waypoints = new Waypoint[102];
            waypoints[0] = new StartWaypoint(start);
            waypoints[waypoints.length - 1] = new EndWaypoint(end,
                    movementSpeed, turnSpeed, followRadius, positionBuffer, rotationBuffer);

            for (int i = 0; i < 100; i++)
            {
                waypoints[i + 1] = new GeneralWaypoint(
                        followPathTrajectory.sample(((i / 100.0) * seconds)).poseMeters,
                        movementSpeed, turnSpeed, followRadius);
            }


            // follow path formed by waypoints
            PurePursuitCommand command = new PurePursuitCommand(
                    robot.drive, robot.odometry, waypoints);
            RunCommand(command, "follow path", waypoints, followPathTrajectory);
        }
    }

    // returns to origin position with pure pursuit
    public void ReturnHome() {

        // check that program is running
        if (opModeIsActive()) {

            // debug
            DebugPartial("return home");

            // create start and end waypoints from current pose to origin pose
            Pose2d start = robot.odometry.getPose();
            Pose2d end = new Pose2d();

            returnHomeInteriorWaypoints.add(new Translation2d(0, 0));
            positiveStartingX = (start.getX() > 0);
            positiveStartingY = (start.getY() > 0);

            Translation2d last = returnHomeInteriorWaypoints.get(1);
            Pose2d waypointPose2d = new Pose2d(last.getX(), last.getY(), new Rotation2d());
            GeneralWaypoint finalWaypoint = new GeneralWaypoint(waypointPose2d, movementSpeed,
                    turnSpeed, followRadius);

            TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);
            config.setReversed(DetectReverse(positiveStartingY, finalWaypoint));

            Trajectory returnHomeTrajectory = TrajectoryGenerator.generateTrajectory(
                    start, returnHomeInteriorWaypoints, end, config);

            // follow path formed by waypoints
            double seconds = returnHomeTrajectory.getTotalTimeSeconds();
            Waypoint[] waypoints = new Waypoint[102];
            waypoints[0] = new StartWaypoint(start);
            waypoints[waypoints.length - 1] = new EndWaypoint(end, movementSpeed,
                    turnSpeed, followRadius, positionBuffer, rotationBuffer);

            for (int i = 0; i < 100; i++)
            {
                waypoints[i + 1] = new GeneralWaypoint(
                        returnHomeTrajectory.sample(((i / 100.0) * seconds)).poseMeters,
                        movementSpeed, turnSpeed, followRadius);
            }

            PurePursuitCommand command = new PurePursuitCommand(
                    robot.drive, robot.odometry, waypoints);
            RunCommand(command, "return home", waypoints, returnHomeTrajectory);

        }
    }

    // run command linearly
    public void RunCommand(PurePursuitCommand command, String state, Waypoint[] waypoints, Trajectory trajectory) {
        // follow path
        command.schedule();

        // loop while following
        while (opModeIsActive() && !command.isFinished()) {

            command.execute();
            robot.odometry.update();
            DebugFull(state);
        }

        command.end(true);
        robot.drive.stop();


        // wait a second
        if (opModeIsActive()) sleep(1000);
    }

    public boolean DetectReverse(boolean positiveY, Waypoint lastWaypoint)
    {
        double yPos = lastWaypoint.getPose().getY();
        double currentPosition = robot.odometry.getPose().getY();
        double currentRotation = robot.odometry.getPose().getRotation().getDegrees();

        if(positiveY)
        {
            if(yPos < currentPosition)
            {
                if(currentRotation <= 90.0 && currentRotation >= -90.0)
                {
                    return true;
                }
            }
        }

        else
        {
            if(yPos > currentPosition)
            {
                if(currentRotation >= 90.0 || currentRotation <= -90.0)
                {
                    return true;
                }
            }
        }

        return false;
    }
    
    /*public double getProgress(Waypoint[] waypoints, Trajectory trajectory)
    {
        double seconds = trajectory.getTotalTimeSeconds();
        double elapsed = 0.0;

        Waypoint winnerOfForEach = null;
        double offset = 10.0;

        double xPos = robot.odometry.getPose().getX();
        double yPos = robot.odometry.getPose().getY();
        Translation2d translation = new Translation2d(xPos, yPos);

        for (Waypoint w : waypoints)
        {
            double waypointX = w.getPose().getX();
            double waypointY = w.getPose().getY();
            Translation2d waypointTranslation = new Translation2d(waypointX, waypointY);

            if(distanceFormula(waypointTranslation, translation) >= offset)
            {
                winnerOfForEach = w;
                offset = distanceFormula(waypointTranslation, translation);
            }
        }

        for (double i = 0.0; i < seconds; i++)
        {
            if(winnerOfForEach == trajectory.sample(i))
            {
                elapsed = i;
                break;
            }
        }

        return elapsed;
    }*/

    public double distanceFormula(Translation2d a, Translation2d b)
    {
        double initial = Math.pow((a.getX() - b.getX()), 2);
        double second = Math.pow((a.getY() - b.getY()), 2);
        return Math.sqrt((initial + second));
    }

    // debug program state with telemetry
    public void DebugPartial(String state) {

        telemetry.addData("state", state);
        telemetry.update();
    }

    // debug info on bot with telemetry
    public void DebugFull(String state) {

        telemetry.addData("state", state);
        telemetry.addData("point count", recording.size());
        telemetry.addData("current pose", robot.odometry.getPose());
        telemetry.addData("encoder vertical left", robot.encoderL.getAsDouble());
        telemetry.addData("encoder vertical right", robot.encoderR.getAsDouble());
        telemetry.addData("encoder horizontal", robot.encoderH.getAsDouble());
        telemetry.addData("input vertical", gamepad1.left_stick_x);
        telemetry.addData("input horizontal", -gamepad1.left_stick_y);
        telemetry.addData("input rotational", gamepad1.right_stick_x);
        telemetry.update();
    }

    /*public void DebugTrajectory(String state, Trajectory trajectory, Waypoint[] waypoints)
    {
        telemetry.addData("state", state);
        telemetry.addData("total trajectory time", trajectory.getTotalTimeSeconds());
        telemetry.addData("elapsed time", getProgress(waypoints, trajectory));
    }*/
}
