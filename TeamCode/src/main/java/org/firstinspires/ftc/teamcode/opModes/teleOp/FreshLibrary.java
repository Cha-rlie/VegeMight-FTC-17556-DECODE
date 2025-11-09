package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.OpModeReference;

@TeleOp(name="TELEOP")
@Configurable
public class FreshLibrary extends CommandOpMode {
    GamepadEx gamePad1;
    GamepadEx gamePad2;

    public static boolean drive = true;
    public static boolean visionTesting = false;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        gamePad1 = new GamepadEx(gamepad1);
        gamePad2 = new GamepadEx(gamepad2);

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
        gamePad1.getGamepadButton(GamepadKeys.Button.A).whenPressed(OpModeReference.getInstance().outtakeSubSystem.shoot());
        //gamePad2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(OpModeReference.getInstance().globalsSubSystem.setRobotStateCommand(RobotState.PARKASCENT));

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        if (drive) {OpModeReference.getInstance().driveTrainSubSystem.driveRobotCentric(gamePad1.getLeftX(), gamePad1.getLeftY(), gamePad1.getRightX());}
        telemetry.update();
    }
}
