package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Disabled
@TeleOp(name="TELEOP")
@Configurable
public abstract class   FreshLibrary extends CommandOpMode {
    GamepadEx gamePad1;
    GamepadEx gamePad2;

    public static boolean drive = true;
    public static boolean visionTesting = false;

    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    public boolean endAutoPathing = true;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        gamePad1 = new GamepadEx(gamepad1);
        gamePad2 = new GamepadEx(gamepad2);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(56, 14))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                .build();

        OpModeReference.getInstance().isRedAlliance = this.getClass().getSimpleName().contains("Red");

        OpModeReference.getInstance().initHardware(hardwareMap, gamePad1, gamePad2, telemetry, 0, 0, 0, visionTesting);

        // TRIANGLE = Y
        // CIRCLE = B
        // CROSS = A
        // SQUARE = X

        // ---------------- BUTTON BINDINGS ------------------- //
        // State Machine
        gamePad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(OpModeReference.getInstance().globalsSubSystem.forwardsRobotState());
        gamePad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(OpModeReference.getInstance().globalsSubSystem.backwardsRobotState());
        gamePad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(OpModeReference.getInstance().globalsSubSystem.forwardsRobotState());
        gamePad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(OpModeReference.getInstance().globalsSubSystem.backwardsRobotState());
        gamePad1.getGamepadButton(GamepadKeys.Button.A).whenPressed(OpModeReference.getInstance().outtakeSubSystem.toggleContinuousShoot());
        gamePad1.getGamepadButton(GamepadKeys.Button.B).whenPressed(OpModeReference.getInstance().outtakeSubSystem.shootBackTriangle());
        gamePad1.getGamepadButton(GamepadKeys.Button.X).whenPressed(OpModeReference.getInstance().outtakeSubSystem.shootFrontTriangle());
        gamePad2.getGamepadButton(GamepadKeys.Button.B).whenPressed(OpModeReference.getInstance().outtakeSubSystem.shootBackTriangle());
        gamePad2.getGamepadButton(GamepadKeys.Button.X).whenPressed(OpModeReference.getInstance().outtakeSubSystem.shootFrontTriangle());
        gamePad1.getGamepadButton(GamepadKeys.Button.A).whenPressed(OpModeReference.getInstance().outtakeSubSystem.shoot());
        gamePad2.getGamepadButton(GamepadKeys.Button.A).whenPressed(OpModeReference.getInstance().outtakeSubSystem.shoot());
        //gamePad2.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(OpModeReference.getInstance().outtakeSubSystem.alignTurret());
        gamePad1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(()->{
            follower.followPath(pathChain.get());
            automatedDrive=true;
            endAutoPathing=false;
        }));
        gamePad2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(()-> endAutoPathing=true));
        gamePad2.getGamepadButton(GamepadKeys.Button.START).whenPressed((OpModeReference.getInstance().flagSubsystem.toggleFlag()));
        gamePad1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).and(gamePad1.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(OpModeReference.getInstance().globalsSubSystem.toggleDefense()));

        // Emergencies
        // Reset bot pose
        gamePad1.getGamepadButton(GamepadKeys.Button.BACK).and(gamePad1.getGamepadButton(GamepadKeys.Button.START).whenPressed(OpModeReference.getInstance().pedroPathing.resetBotPose()));
        gamePad2.getGamepadButton(GamepadKeys.Button.BACK).and(gamePad1.getGamepadButton(GamepadKeys.Button.START).whenPressed(OpModeReference.getInstance().pedroPathing.resetBotPose()));
        // Spit stuck balls out backwards
        gamePad2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(OpModeReference.getInstance().intakeSubSystem.spitOut());
        gamePad2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(OpModeReference.getInstance().outtakeSubSystem.toggleTurretAlignmentInput());
    }

    /* Gamepad 2
    LEFT BUMPER -> Go backwards in state machine
    RIGHT BUMPER -> Go forwards in state machine
    B -> Shoot back triangle
    X -> Shoot front triangle
    DPAD Down -> Transfer Backwards
    DPAD Up -> Transger Up
    START -> FLAGGGGG!!!
    BACK -> Stuck Shoot
    RIGHT JOYSTICK X VALUE -> Manual turret control

     */

    public abstract void initalize();

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        if (!automatedDrive) {
            if (drive) {
                OpModeReference.getInstance().driveTrainSubSystem.driveRobotCentric(gamePad1.getLeftX(), gamePad1.getLeftY(), gamePad1.getRightX());
            }
            telemetry.update();
        } else {
            if (automatedDrive && (!follower.isBusy() || endAutoPathing)) {
                endAutoPathing=false;
                automatedDrive=false;
                follower.breakFollowing();
            }
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
        OpModeReference.getInstance().limelightSubsystem.closeLimeLight().schedule();
        reset();
    }
}
