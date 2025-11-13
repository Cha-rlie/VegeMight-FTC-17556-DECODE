//package org.firstinspires.ftc.teamcode.opModes.auto;
//
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.common.OpModeReference;
//
//@Autonomous(name="Six Artifact Auto")
//public class sixArtifactAuto extends CommandOpMode {
//    // Initialise the PedroPathing Follower
//    private boolean once = true;
//    private Follower follower;
//
//    /** This is the variable where we store the state of our auto.
//     * It is used by the pathUpdate method. */
//    private int pathState;
//    private Timer pathTimer, actionTimer, opmodeTimer;
//
//    // Initialise the poses
//    private final Pose startPose = new Pose(9.000, 113, Math.toRadians(0));  // Starting position
//    private final Pose scorePose = new Pose(14, 128, Math.toRadians(-45)); // Scoring position
//
//    private final Pose pickup1Pose = new Pose(24, 121.5, Math.toRadians(0)); // First sample pickup
//    private final Pose pickup2Pose = new Pose(24, 131.5, Math.toRadians(0)); // Second sample pickup
//    private final Pose pickup3Pose = new Pose(24, 135, Math.toRadians(30)); // Third sample pickup
//
//    private final Pose parkPose = new Pose(60, 95, Math.toRadians(90));    // Parking position
//    private final Pose parkControlPose = new Pose(60, 120); // Control point for curved path
//    // Declare paths and pathchains
//    private Path scorePreload;
//    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, park;
//    private boolean readyForNext = true;
//
//@Override
//public void run() {
//    if (once) {
//        OpModeReference.getInstance().globalsSubSystem.goToIdle();
//        opmodeTimer.resetTimer();
//        setPathState(0);
//        once = false;
//    }
//    // These loop the movements of the robot
//    CommandScheduler.getInstance().run();
//    follower.update();
//    autonomousPathUpdate();
//
//    // Feedback to Driver Hub
//    telemetry.addData("path state", pathState);
//    telemetry.addData("x", follower.getPose().getX());
//    telemetry.addData("y", follower.getPose().getY());
//    telemetry.addData("heading", follower.getPose().getHeading());
//    telemetry.addData("IsBusy?", OpModeReference.getInstance().isBusy());
//    telemetry.update();
//}
//
//@Override
//public void initialize() {
//    readyForNext = true;
//    CommandScheduler.getInstance().reset();
//    OpModeReference.getInstance().initHardware(hardwareMap, new GamepadEx(gamepad1), new GamepadEx(gamepad2), telemetry, 0, 0 ,0);
//    OpModeReference.getInstance().globalsSubSystem.goToInit().schedule();
//
//    pathTimer = new Timer();
//    opmodeTimer = new Timer();
//    opmodeTimer.resetTimer();
//
//    Constants.setConstants(FConstants.class, LConstants.class);
//    follower = new Follower(hardwareMap, FConstants.class, LConstants.class); /* Check Later */
//    follower.setStartingPose(startPose);
//    follower.setMaxPower(0.7);
//    buildPaths();
//    while (opModeInInit()) {
//        CommandScheduler.getInstance().run();
//    }
//
//}
//
//}
