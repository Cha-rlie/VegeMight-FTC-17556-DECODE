package org.firstinspires.ftc.teamcode.opModes.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.common.util.RobotState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name="BLUE 9 Artifact", group="9 Auto")
public class BlueNineArtifactAuto extends CommandOpMode {

    // Initialise the PedroPathing Follower
    private boolean once = true;
    private Follower follower;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private Timer pathTimer, actionTimer, opmodeTimer;

    // Initialise the poses

    private final Pose startPose = new Pose(144-87, 10,Math.toRadians(-90));
    private final Pose leavePose = new Pose(144-16, -16, Math.toRadians(-90));

    private PathChain Path1, Path2, Path3, Path4, PathLeave;
    private boolean readyForNext = false;
    public boolean shooting = true;
    String currentState = "Shooting";

    public void buildPaths() {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-87.000, 10.000),
                                new Pose(144-84.444, 48.000),
                                new Pose(144-85.556, 35.778),
                                new Pose(144-135.111, 36.222)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-180))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-135.111, 36.222), new Pose(144-84.667, 11.556))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-90))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-84.667, 11.556),
                                new Pose(144-78.000, 63.333),
                                new Pose(144-75.333, 63.333),
                                new Pose(144-135.333, 59.333)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-180))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-135.333, 59.333), new Pose(144-84.667, 11.778))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-90))
                .build();

        PathLeave = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-84.667, 11.778), new Pose(144-84.667, 56))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to scoring position
                // Shoot 3 + 1 for safety bonus
                currentState = "Shooting preloads from back triangle";
                if (readyForNext) {
                    new SequentialCommandGroup(
                            new InstantCommand((()->readyForNext = false)),
                            OpModeReference.getInstance().outtakeSubSystem.shootBackTriangle(),
                            OpModeReference.getInstance().globalsSubSystem.setRobotStateCommand(RobotState.OUTTAKE),
                            new WaitCommand(2000),
                            OpModeReference.getInstance().outtakeSubSystem.shoot(),
                            new WaitCommand(1000),
                            OpModeReference.getInstance().globalsSubSystem.setRobotStateCommand(RobotState.INTAKE),
                            new InstantCommand(() -> readyForNext = true),
                            new InstantCommand(()->setPathState(1))
                    ).schedule();
                }
                break;
            case 1:
                if (readyForNext && !follower.isBusy()) { // Go to intake position and intake at the same time
                    currentState = "Going to Intake";
                    follower.followPath(Path1);
                    setPathState(2);
                }
                break;
            case 2: // Return to scoring position
                if (readyForNext && (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 1.0)) {
                    OpModeReference.getInstance().globalsSubSystem.setRobotStateCommand(RobotState.TRAVEL);
                    currentState = "Going to Scoring Pose";
                    follower.followPath(Path2);
                    setPathState(3);
                }
                break;
            case 3:
                // Shoot 3 + 1 for safety bonus
                if (readyForNext && (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2)) {
                    new SequentialCommandGroup(
                            new InstantCommand(()-> follower.pausePathFollowing()),
                            new InstantCommand(()->readyForNext=false),
                            new InstantCommand(() -> currentState = "Shooting"),
                            OpModeReference.getInstance().outtakeSubSystem.shootBackTriangle(),
                            OpModeReference.getInstance().globalsSubSystem.setRobotStateCommand(RobotState.OUTTAKE),
                            new WaitCommand(1500),
                            OpModeReference.getInstance().outtakeSubSystem.shoot(),
                            new WaitCommand(1000),
                            OpModeReference.getInstance().globalsSubSystem.setRobotStateCommand(RobotState.INTAKE),
                            new InstantCommand(() -> readyForNext = true),
                            new InstantCommand(()->setPathState(4))
                    ).schedule();
                }
                break;
            case 4:
                if (readyForNext) { // Go to intake position and intake at the same time
                    currentState = "Going to Intake";
                    follower.followPath(Path3);
                    setPathState(5);
                }
                break;
            case 5: // Return to scoring position
                if (readyForNext && (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0)) {
                    OpModeReference.getInstance().globalsSubSystem.setRobotStateCommand(RobotState.TRAVEL);
                    currentState = "Going to Scoring Pose";
                    follower.followPath(Path4);
                    setPathState(6);
                }
                break;
            case 6:
                //
                if (readyForNext && (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.6)) {
                    new SequentialCommandGroup(
                            new InstantCommand(()->readyForNext=false),
                            new InstantCommand(() -> currentState = "Shooting"),
                            OpModeReference.getInstance().outtakeSubSystem.shootBackTriangle(),
                            OpModeReference.getInstance().globalsSubSystem.setRobotStateCommand(RobotState.OUTTAKE),
                            new WaitCommand(1500),
                            OpModeReference.getInstance().outtakeSubSystem.shoot(),
                            new WaitCommand(1000),
                            OpModeReference.getInstance().globalsSubSystem.setRobotStateCommand(RobotState.INTAKE),
                            new InstantCommand(() -> readyForNext = true),
                            new InstantCommand(()->setPathState(7))
                    ).schedule();
                }
                break;
            case 7:
                if (readyForNext && (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5)) {
                    // Put flag back down when we get to leave point
                    currentState = "Going to leave point";
                    OpModeReference.getInstance().flagSubsystem.toggleFlag().schedule();
                    follower.followPath(PathLeave);
                    setPathState(8);
                }
                break;
            case 8:
                if (readyForNext && (pathTimer.getElapsedTimeSeconds() > 2 || !follower.isBusy())) {
                    // Put flag back down when we get to leave point
                    currentState = "Arrived at leave point";
                    setPathState(-1);
                }
            default:
                break;
        }
    }

    @Override
    public void run() {
        if (once) {
            opmodeTimer.resetTimer();
            setPathState(0);
            once = false;
        }
        // These loop the movements of the robot
        CommandScheduler.getInstance().run();
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Stage", pathState);
        telemetry.addData("IsBusy?", follower.isBusy());
        telemetry.addData("Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("CurrentState", currentState);
        telemetry.update();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void initialize() {
        readyForNext = true;
        OpModeReference.getInstance().updateGlobalRobotPose(startPose);
        CommandScheduler.getInstance().reset();
        OpModeReference.getInstance().isRedAlliance = false;
        OpModeReference.getInstance().initHardware(hardwareMap, new GamepadEx(gamepad1), new GamepadEx(gamepad2), telemetry, 0, 0, 0, false);
        OpModeReference.getInstance().globalsSubSystem.goToInit().schedule();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.9);
        buildPaths();

        while (opModeInInit()) {
            CommandScheduler.getInstance().run();
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
        OpModeReference.getInstance().updateGlobalRobotPose(follower.getPose());
        OpModeReference.getInstance().limelightSubsystem.closeLimeLight().schedule();
        reset();
    }

}
