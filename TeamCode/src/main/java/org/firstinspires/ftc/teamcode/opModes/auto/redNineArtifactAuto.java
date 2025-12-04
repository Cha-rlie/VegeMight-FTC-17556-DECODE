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

@Autonomous(name="🟥 Red Nine Artifact Auto")
public class redNineArtifactAuto extends CommandOpMode {
    // Initialise the PedroPathing Follower
    private boolean once = true;
    private Follower follower;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private Timer pathTimer, actionTimer, opmodeTimer;

    // Initialise the poses

    private final Pose startPose = new Pose(86, 14,Math.toRadians(-90));
    private final Pose leavePose = new Pose(16, -16, Math.toRadians(-90));

    private PathChain Path1, Path2, Path3, Path4, PathLeave;
    private boolean readyForNext = false;
    public boolean shooting = true;
    String currentState = "Shooting";

    public void buildPaths() {

            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.222, 9.111),
                                    new Pose(62.000, 41.556),
                                    new Pose(70.000, 34.889),
                                    new Pose(10.000, 36.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(10.000, 36.000), new Pose(56.222, 12.111))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(70))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.222, 12.111),
                                    new Pose(66.444, 64.666),
                                    new Pose(10.000, 58.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(10.000, 58.000), new Pose(56.222, 12.11))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .build();

        PathLeave = follower.pathBuilder()
                .addPath(
                        new BezierLine(startPose, new Pose(56, -50))
                ).setConstantHeadingInterpolation(Math.toRadians(-90)).build();


        Path1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(86.000, 9.000),
                                new Pose(80, 45),
                                new Pose(130.8, 35.7)
                        )
                )
                .setTangentHeadingInterpolation().build();

        Path2 = follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(9.697, -36.364), new Pose(56.000, -9.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                        .build();

        Path3 = follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(56.00, -9.00), leavePose))
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
                            new WaitCommand(1000),
                            OpModeReference.getInstance().outtakeSubSystem.shoot(),
                            new WaitCommand(2000),
                            OpModeReference.getInstance().globalsSubSystem.setRobotStateCommand(RobotState.INTAKE),
                            new InstantCommand(() -> readyForNext = true),
                            setPathState(1)
                    ).schedule();
                }
                break;
            case 1:
                if (readyForNext) { // Go to intake position and intake at the same time
                    currentState = "Going to Intake";
                    follower.followPath(Path1);
                    setPathState(2);
                }
                break;
            case 2: // Return to scoring position
                if (!follower.isBusy() && readyForNext /*&& pathTimer.getElapsedTimeSeconds() > 3.5*/) {
                    OpModeReference.getInstance().globalsSubSystem.setRobotStateCommand(RobotState.TRAVEL);
                    currentState = "Going to Scoring Pose";
                    follower.followPath(Path2);
                    setPathState(3);
                }
                break;
            case 3:
                // Shoot 3 + 1 for safety bonus
                if (!follower.isBusy() && readyForNext /*&& pathTimer.getElapsedTimeSeconds() > 3.5*/) {
                    new SequentialCommandGroup(
                        new InstantCommand(()->readyForNext=false),
                        new InstantCommand(() -> currentState = "Shooting"),
                        OpModeReference.getInstance().outtakeSubSystem.shootBackTriangle(),
                        OpModeReference.getInstance().globalsSubSystem.setRobotStateCommand(RobotState.OUTTAKE),
                        new WaitCommand(1000),
                        OpModeReference.getInstance().outtakeSubSystem.shoot(),
                        new WaitCommand(2000),
                        OpModeReference.getInstance().globalsSubSystem.setRobotStateCommand(RobotState.INTAKE),
                        new InstantCommand(() -> readyForNext = false),
                        setPathState(4)
                    ).schedule();
                }
                break;
            case 4:
                if (!follower.isBusy() && readyForNext) { // Go to intake position and intake at the same time
                    currentState = "Going to Intake";
                    follower.followPath(Path3);
                    setPathState(5);
                }
                break;
            case 5: // Return to scoring position
                if (!follower.isBusy() && readyForNext /*&& pathTimer.getElapsedTimeSeconds() > 3.5*/) {
                    OpModeReference.getInstance().globalsSubSystem.setRobotStateCommand(RobotState.TRAVEL);
                    currentState = "Going to Scoring Pose";
                    follower.followPath(Path4);
                    setPathState(6);
                }
                break;
            case 6:
                // Shoot 3 + 1 for safety bonus
                if (!follower.isBusy() && readyForNext /*&& pathTimer.getElapsedTimeSeconds() > 3.5*/) {
                    new SequentialCommandGroup(
                            new InstantCommand(()->readyForNext=false),
                            new InstantCommand(() -> currentState = "Shooting"),
                            OpModeReference.getInstance().outtakeSubSystem.shootBackTriangle(),
                            OpModeReference.getInstance().globalsSubSystem.setRobotStateCommand(RobotState.OUTTAKE),
                            new WaitCommand(1000),
                            OpModeReference.getInstance().outtakeSubSystem.shoot(),
                            new WaitCommand(2000),
                            OpModeReference.getInstance().globalsSubSystem.setRobotStateCommand(RobotState.INTAKE),
                            new InstantCommand(() -> readyForNext = false),
                            setPathState(7)
                    ).schedule();
                }
                break;
            case 7:
                if (!follower.isBusy() && readyForNext /*&& pathTimer.getElapsedTimeSeconds() > 2*/) {
                    // Put flag back down when we get to leave point
                    currentState = "Going to leave point";
                    OpModeReference.getInstance().flagSubsystem.toggleFlag().schedule();
                    follower.followPath(PathLeave);
                    setPathState(8).schedule();
                }
                break;
            case 8:
                if (!follower.isBusy() && readyForNext /*&& pathTimer.getElapsedTimeSeconds() > 2*/) {
                    // Put flag back down when we get to leave point
                    currentState = "Arrived at leave point";
                    setPathState(-1).schedule();
                }
            default:
                break;
        }
    }

    /*private final Pose startPose = new Pose(9.000, 113, Math.toRadians(0));  // Starting position
    private final Pose scorePose = new Pose(14, 128, Math.toRadians(-45)); // Scoring position

    private final Pose pickup1Pose = new Pose(24, 121.5, Math.toRadians(0)); // First sample pickup
    private final Pose pickup2Pose = new Pose(24, 131.5, Math.toRadians(0)); // Second sample pickup
    private final Pose pickup3Pose = new Pose(24, 135, Math.toRadians(30)); // Third sample pickup

    private final Pose parkPose = new Pose(60, 95, Math.toRadians(90));    // Parking position
    private final Pose parkControlPose = new Pose(60, 120); // Control point for curved path
    // Declare paths and pathchains
    private Path scorePreload;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, park;
   */

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

    public InstantCommand setPathState(int pState) {
        return new InstantCommand(()->{
            pathState = pState;
            pathTimer.resetTimer();
        });
    }

    @Override
    public void initialize() {
        readyForNext = true;
        CommandScheduler.getInstance().reset();
        OpModeReference.getInstance().isRedAlliance = true;
        OpModeReference.getInstance().initHardware(hardwareMap, new GamepadEx(gamepad1), new GamepadEx(gamepad2), telemetry, 0, 0, 0, false);
        OpModeReference.getInstance().globalsSubSystem.goToInit().schedule();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.7);
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
