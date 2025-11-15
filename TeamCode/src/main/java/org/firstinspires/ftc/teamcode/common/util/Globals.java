package org.firstinspires.ftc.teamcode.common.util;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.OpModeReference;

import java.util.HashMap;


public class Globals extends SubsystemBase {

    public static final Globals INSTANCE = new Globals();
    public static boolean isSampleModeTrue = true;
    public boolean updateRobotStateTrue = true;
    public boolean intakeAcceptState = false;

    // Declare the global variables
    private RobotState robotState;
    public RobotState lastRobotState;

    public boolean robotAutomaticallyMoving;
    // TODO: make poses and location enum
    //public XX targetLocation;
    public boolean angledForShooting;
    public boolean positionForShooting;

    UpdateAndPowerScheduler updateAndPowerScheduler;

    private HashMap <RobotState, RobotState> goForwardStateValuesOnly;
    private HashMap <RobotState, RobotState> goBackwardStateValuesOnly;

    // Constructor that builds the drivetrain subsystem class and Hashmaps :D
    public Globals() {
        goForwardStateValuesOnly = new HashMap<RobotState, RobotState>() {{
            put(RobotState.TRANSFER, RobotState.OUTTAKE);
            put(RobotState.OUTTAKE, RobotState.TRANSFER);
            put(RobotState.INTAKE, RobotState.TRANSFER);
            put(RobotState.INIT, RobotState.TRANSFER);
        }};
        goBackwardStateValuesOnly = new HashMap<RobotState, RobotState>() {{
            put(RobotState.TRANSFER, RobotState.INTAKE);
            put(RobotState.OUTTAKE, RobotState.TRANSFER);
            put(RobotState.INTAKE, RobotState.TRANSFER);
            put(RobotState.INIT, RobotState.TRANSFER);
        }};

        robotState = RobotState.TRANSFER;
        lastRobotState = RobotState.TRANSFER;

        updateAndPowerScheduler = OpModeReference.getInstance().updateAndPowerScheduler;
    }

    public RobotState getRobotState() {
        return robotState;
    }

    public InstantCommand setRobotStateCommand(RobotState newRobotState) {
        return new InstantCommand(()-> {
            lastRobotState = robotState;
            robotState = newRobotState;
            //updateAndPowerScheduler.robotUpdate=true;
        });
    }

    @NonNull
    public InstantCommand forwardsRobotState() {
        return new InstantCommand(()-> {
            robotState = goForwardStateValuesOnly.get(getRobotState());
            //updateAndPowerScheduler.robotUpdate=true;
        });
    }

    @NonNull
    public InstantCommand toggleDefense(){
        return new InstantCommand(()->{
            if (getRobotState()==RobotState.DEFENSE) {
                robotState = RobotState.OUTTAKE;
            } else {
                robotState = RobotState.DEFENSE;
            }
        });
    }

    @NonNull
    public InstantCommand backwardsRobotState() {
        return new InstantCommand(()-> {
            robotState = goBackwardStateValuesOnly.get(getRobotState());
            //updateAndPowerScheduler.robotUpdate=true;
        });
    }

    @NonNull
    public InstantCommand goToInit(){
        return new InstantCommand(()->{
           robotState=RobotState.INIT;
        });
    }

    @NonNull
    public InstantCommand goToTRANSFER() {
        return new InstantCommand(()-> {
            if (robotState != RobotState.TRANSFER) {
                robotState = RobotState.TRANSFER;
            }
            updateAndPowerScheduler.robotUpdate=true;
        });
    }


    @NonNull
    public InstantCommand reject() {
        return new InstantCommand(()->{
            robotState= RobotState.REJECT;
            updateAndPowerScheduler.robotUpdate=true;
        });
    }


    @Override
    public void periodic() {
        OpModeReference.getInstance().getTelemetry().addData("Robot State", robotState);
        OpModeReference.getInstance().getTelemetry().addData("Update Robot State", updateRobotStateTrue);
        OpModeReference.getInstance().getTelemetry().addData("isSampleTrue", isSampleModeTrue);
    }

}