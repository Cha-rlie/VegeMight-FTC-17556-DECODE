package org.firstinspires.ftc.teamcode.opModes.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
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

@Autonomous(name="🟥 Red Six Artifact Auto")
public class redSixArtifactAuto extends CommandOpMode {
    // Initialise the PedroPathing Follower
    private boolean once = true;
    private Follower follower;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private Timer pathTimer, actionTimer, opmodeTimer;

    // Initialise the poses

    private final Pose startPose = new Pose(56, -14,Math.toRadians(-90));
    private final Pose leavePose = new Pose(16, -16, Math.toRadians(-90));

    private PathChain Path1, Path2, Path3, PathLeave;
    private boolean readyForNext = false;
    public boolean shooting = true;
    String currentState = "Shooting";

    public void buildPaths() {

        PathLeave = follower.pathBuilder()
                .addPath(
                        new BezierLine(startPose, new Pose(56, -50))
                ).setConstantHeadingInterpolation(Math.toRadians(-90)).build();


        Path1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, -9.000),
                                new Pose(51.394, -40.727),
                                new Pose(9.697, -36.364)
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
                if (readyForNext) {
                    readyForNext = false;
                    OpModeReference.getInstance().outtakeSubSystem.shootBackTriangle()
                            .andThen(new InstantCommand(()-> readyForNext=false))
                            .andThen(OpModeReference.getInstance().outtakeSubSystem.turretTurnToPos(280))
                            .andThen(new InstantCommand(()->OpModeReference.getInstance().outtakeSubSystem.flywheel.setVelocity(800)))
                            .andThen(new WaitCommand(700))
                            .andThen(OpModeReference.getInstance().globalsSubSystem.setRobotStateCommand(RobotState.OUTTAKE)
                            .andThen(new InstantCommand(()->shooting=true)))
                            .andThen(new WaitCommand(2000))
                            .andThen(OpModeReference.getInstance().outtakeSubSystem.autonomousShoot())
                            .andThen(new WaitCommand(200))
                            .andThen(OpModeReference.getInstance().outtakeSubSystem.autonomousShoot())
                            .andThen(new WaitCommand(200))
                            .andThen(OpModeReference.getInstance().outtakeSubSystem.autonomousShoot())
                            .andThen(new WaitCommand(200))
                            .andThen(OpModeReference.getInstance().outtakeSubSystem.autonomousShoot())
                            .andThen(new WaitCommand(200))
                            .andThen(new InstantCommand(()->shooting=false))
                            .andThen(OpModeReference.getInstance().globalsSubSystem.setRobotStateCommand(RobotState.INTAKE))
                            .andThen(new InstantCommand(()->setPathState(1)))
                            .andThen(new InstantCommand(()->readyForNext=true)).schedule();
                }
                break;
            case 1:
                if (!shooting && readyForNext) { // Go to intake position and intake at the same time
                    currentState = "Going to Intake";
                    follower.followPath(PathLeave);
                    setPathState(2);
                }
                break;
            case 2: // Return to scoring position
                if (!follower.isBusy() && readyForNext && pathTimer.getElapsedTimeSeconds() > 3.5) {
                    //OpModeReference.getInstance().globalsSubSystem.setRobotStateCommand(RobotState.OUTTAKE);
                    currentState = "Going to Scoring Pose";
                    follower.followPath(Path2);
                    follower.update();
                    setPathState(3);
                }
                break;
            case 3:
                // Shoot 3 + 1 for safety bonus
                if (!shooting && !follower.isBusy() && readyForNext && pathTimer.getElapsedTimeSeconds() > 3.5) {
                    readyForNext = false;
                    OpModeReference.getInstance().outtakeSubSystem.shootBackTriangle()
                            .andThen(new InstantCommand(()->currentState="Shooting"))
                            .andThen(new InstantCommand(()-> readyForNext=false))
                            .andThen(OpModeReference.getInstance().outtakeSubSystem.turretTurnToPos(250))
                            .andThen(new InstantCommand(()->OpModeReference.getInstance().outtakeSubSystem.flywheel.setVelocity(800)))
                            .andThen(new WaitCommand(700))
                            .andThen(OpModeReference.getInstance().globalsSubSystem.setRobotStateCommand(RobotState.OUTTAKE)
                            .andThen(new InstantCommand(()->shooting=true)))
                            .andThen(new WaitCommand(1000))
                            .andThen(OpModeReference.getInstance().outtakeSubSystem.autonomousShoot())
                            .andThen(new WaitCommand(200))
                            .andThen(OpModeReference.getInstance().outtakeSubSystem.autonomousShoot())
                            .andThen(new WaitCommand(200))
                            .andThen(OpModeReference.getInstance().outtakeSubSystem.autonomousShoot())
                            .andThen(new WaitCommand(200))
                            .andThen(OpModeReference.getInstance().outtakeSubSystem.autonomousShoot())
                            .andThen(new WaitCommand(200))
                            .andThen(new InstantCommand(()->shooting=false))
                            .andThen(OpModeReference.getInstance().globalsSubSystem.setRobotStateCommand(RobotState.INTAKE))
                            .andThen(new InstantCommand(()->setPathState(4)))
                            .andThen(new InstantCommand(()->readyForNext=true)).schedule();

                }
                break;
            case 4:
                if (!follower.isBusy() && readyForNext) {
                    // leave points
                    currentState = "Going to leave point";
                    follower.followPath(Path3);
                    follower.update();
                    OpModeReference.getInstance().flagSubsystem.toggleFlag().schedule();
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && readyForNext && pathTimer.getElapsedTimeSeconds() > 2) {
                    // Put flag back down when we get to leave point
                    currentState = "Arrived at end point";
                    OpModeReference.getInstance().flagSubsystem.toggleFlag().schedule();
                    setPathState(-1);
                }
                break;
            default:
                break;
        }
        ;

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

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

@Override
public void initialize() {
    readyForNext = true;
    CommandScheduler.getInstance().reset();
    OpModeReference.getInstance().isRedAlliance = true;
    OpModeReference.getInstance().initHardware(hardwareMap, new GamepadEx(gamepad1), new GamepadEx(gamepad2), telemetry, 0, 0 ,0, false);
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
        reset();
    }


}


/* Paths

public static class Paths {

  public PathChain Path1;
  public PathChain Path2;

  public Paths(Follower follower) {
    Path1 = follower
      .pathBuilder()
      .addPath(
        new BezierCurve(
          new Pose(56.000, 9.000),
          new Pose(51.394, 40.727),
          new Pose(9.697, 36.364)
        )
      )
      .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
      .build();

    Path2 = follower
      .pathBuilder()
      .addPath(new BezierLine(new Pose(9.697, 36.364), new Pose(56.000, 9.000)))
      .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
      .build();
  }
}
 */
