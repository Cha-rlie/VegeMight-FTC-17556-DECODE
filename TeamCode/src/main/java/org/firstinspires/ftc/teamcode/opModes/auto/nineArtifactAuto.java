package org.firstinspires.ftc.teamcode.opModes.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Nine Artifact Auto")
public class nineArtifactAuto extends CommandOpMode {
    // Initialise the PedroPathing Follower
    private boolean once = true;
    private Follower follower;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private Timer pathTimer, actionTimer, opmodeTimer;

    // Initialise the poses

    private final Pose startPose = new Pose(56.000, 9.000,Math.toRadians(90));

    private PathChain Path1;
    private PathChain Path2;
    private PathChain Path3;
    private PathChain Path4;
    private boolean readyForNext = true;

    public void buildPaths() {
        Path1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, 9.000),
                                new Pose(51.394, 40.727),
                                new Pose(9.697, 36.364)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        Path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(9.697, 36.364), new Pose(56.000, 9.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();
        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, 9.000),
                                new Pose(69.576, 66.667),
                                new Pose(8.970, 60.121)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(8.970, 60.121), new Pose(56.000, 9.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to scoring position
                follower.followPath(Path1);
                setPathState(1);
                break;

            case 1: // Wait until the robot is near the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(Path2, true);
                    setPathState(-1);
                }
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
        telemetry.addData("IsBusy?", OpModeReference.getInstance().isBusy());
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

    /* Path Chain

    public static class Paths {

  public PathChain Path1;
  public PathChain Path2;
  public PathChain Path3;
  public PathChain Path4;

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

    Path3 = follower
      .pathBuilder()
      .addPath(
        new BezierCurve(
          new Pose(56.000, 9.000),
          new Pose(69.576, 66.667),
          new Pose(8.970, 60.121)
        )
      )
      .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
      .build();

    Path4 = follower
      .pathBuilder()
      .addPath(new BezierLine(new Pose(8.970, 60.121), new Pose(56.000, 9.000)))
      .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
      .build();
  }
}



     */
}
