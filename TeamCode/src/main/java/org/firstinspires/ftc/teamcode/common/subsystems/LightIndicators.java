package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.lights.RGBIndicator;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.common.util.Globals;
import org.firstinspires.ftc.teamcode.common.util.RobotState;

public class LightIndicators extends SubsystemBase {
    Servo leftLight;
    Servo rightLight;
    Globals globals;
    public static double initColour = 0.388; // Yellow
    public static double intakeColour = 0.6; // Blue
    public static double travelColour = 0.29; // Red
    public static double outtakeColour = 0.49; // Green
    public static double defenseColour = 0.7; // Purple

    public LightIndicators() {
        CommandScheduler.getInstance().registerSubsystem(this);
        leftLight = OpModeReference.getInstance().getHardwareMap().get(Servo.class, "LL");
        rightLight = OpModeReference.getInstance().getHardwareMap().get(Servo.class, "RL");

        globals = OpModeReference.getInstance().globalsSubSystem;

        setDefaultCommand(new PerpetualCommand(setLightsBasedOffState()));
    }

    @Override
    public void periodic() {
        //OpModeReference.getInstance().getTelemetry().addData("Current Colour", XX);
    }

    public RunCommand setLightsBasedOffState() {
        return new RunCommand(() -> {
            switch (globals.getRobotState()) {
                case INIT:
                    leftLight.setPosition(initColour);
                    rightLight.setPosition(initColour);
                    break;
                case INTAKE:
                    leftLight.setPosition(intakeColour);
                    rightLight.setPosition(intakeColour);
                    break;
                case TRAVEL:
                    leftLight.setPosition(travelColour);
                    rightLight.setPosition(travelColour);
                    break;
                case OUTTAKE:
                    leftLight.setPosition(outtakeColour);
                    rightLight.setPosition(outtakeColour);
                    break;
                case DEFENSE:
                    leftLight.setPosition(defenseColour);
                    rightLight.setPosition(defenseColour);
                    break;
            }
        }, this);
    }
}
