package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.common.util.Globals;
import org.firstinspires.ftc.teamcode.common.util.RobotState;
import org.firstinspires.ftc.teamcode.common.util.UpdateAndPowerScheduler;

@Configurable
public class Outtake extends SubsystemBase {
    public MotorEx flywheel;
    public MotorEx turret;
    Servo hoodL;
    Servo hoodR;
    Servo flipper;

    public static double flipUpPos = 0.175;
    public static double flipGroundPos = 0.01;
    public static int flywheelVelocity = 1000;
    public static int turretRTP = 0;
    public static int hoodangle = 0;

    Globals globals;
    UpdateAndPowerScheduler updateAndPowerScheduler;

    public Outtake() {
        // Set up hardware
        flywheel = new MotorEx(OpModeReference.getInstance().getHardwareMap(), "FW", Motor.GoBILDA.BARE);
        turret = new MotorEx(OpModeReference.getInstance().getHardwareMap(), "T", Motor.GoBILDA.RPM_312);
        hoodL = OpModeReference.getInstance().getHardwareMap().get(Servo.class, "HL");
        hoodR = OpModeReference.getInstance().getHardwareMap().get(Servo.class, "HR");
        hoodR.setDirection(Servo.Direction.REVERSE);
        flipper = OpModeReference.getInstance().getHardwareMap().get(Servo.class, "F");
        flywheel.setRunMode(Motor.RunMode.VelocityControl);
        turret.setRunMode(Motor.RunMode.PositionControl);
        flywheel.setInverted(true);
        turret.setPositionTolerance(150);
        turret.stopAndResetEncoder();

        //flywheel.setFeedforwardCoefficients(0.05,0.01,0.31);
        //flywheel.setRunMode(Motor.RunMode.RawPower);

        globals = OpModeReference.getInstance().globalsSubSystem;
        updateAndPowerScheduler = OpModeReference.getInstance().updateAndPowerScheduler;
        setDefaultCommand(new PerpetualCommand(defaultCommand()));

    }

    public RunCommand defaultCommand() {
        return new RunCommand(()->{
            if (globals.getRobotState() == RobotState.OUTTAKE) {
                flywheel.setVelocity(flywheelVelocity);
                flywheel.setRunMode(Motor.RunMode.VelocityControl);
                turret.setTargetPosition(turretRTP);
                turret.set(0.4);
                if (turret.atTargetPosition()){
                    turret.set(0);
                };
            } else if (globals.getRobotState() == RobotState.INIT) {
                flywheel.setVelocity(flywheelVelocity*0.5);
                flywheel.setTargetPosition(0);
                hoodL.setPosition(0);
                hoodR.setPosition(0);
                flipper.setPosition(flipGroundPos);
                turret.setTargetPosition(0);
                turret.set(0.4);
                if (turret.atTargetPosition()){
                    turret.set(0);
                };

            } else {
                flywheel.setVelocity(flywheelVelocity*0.5);
            }

//            if (updateAndPowerScheduler.outtakeUpdate) {
//                if (!updateAndPowerScheduler.powerOuttake) {
//                    //disable
//                } else {
//                    //enable
//                }
//                updateAndPowerScheduler.outtakeUpdate=false;
//            }
        }, this);
    }

    public SequentialCommandGroup shoot() {
        return new SequentialCommandGroup(
            new InstantCommand(()->flipper.setPosition(flipUpPos)).andThen(
            new WaitCommand(500)),
            new InstantCommand(()->flipper.setPosition(flipGroundPos)).andThen(
            new WaitCommand(200)).andThen(OpModeReference.getInstance().transfer.transfer().andThen(new WaitCommand(1)))
        );
    }

    @Override
    public void periodic() {
        OpModeReference.getInstance().getTelemetry().addData("Flywheel Velocity", flywheel.getVelocity());
        OpModeReference.getInstance().getTelemetry().addData("Hood Pos", hoodL.getPosition());
        OpModeReference.getInstance().getTelemetry().addData("Flipper Pos", flipper.getPosition());
    }
}
