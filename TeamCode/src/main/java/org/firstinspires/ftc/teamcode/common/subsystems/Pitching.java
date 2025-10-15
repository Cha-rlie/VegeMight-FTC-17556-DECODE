package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.common.util.Globals;
import org.firstinspires.ftc.teamcode.common.util.RobotState;

import java.util.HashMap;

public class Pitching extends SubsystemBase {
    public DcMotorEx pitchingMotor;
    private HashMap<RobotState, Integer> stateToValueMap;
    public int runToPos = 0;
    Globals globals;
    private boolean isBusy;

    public Pitching() {
        isBusy = false;
        CommandScheduler.getInstance().registerSubsystem(this);
        pitchingMotor = OpModeReference.getInstance().getHardwareMap().get(DcMotorEx.class, "P");
        //pitchingMotor = new MotorEx(OpModeReference.getInstance().getHardwareMap(), "P", Motor.GoBILDA.RPM_435);
        pitchingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pitchingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pitchingMotor.setTargetPositionTolerance(50);
        pitchingMotor.setTargetPosition(0);
        pitchingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pitchingMotor.setPower(1);
        runToPos = 0;

        stateToValueMap = new HashMap<RobotState, Integer>() {
            {
                put(RobotState.INIT, 0);
                put(RobotState.IDLE, 0);
                // Add what values for other states

                globals = OpModeReference.getInstance().globalsSubSystem;

                // TODO: Set to a Perpetual Command we know later
                //setDefaultCommand(new PerpetualCommand(turnPitching())); //Perpetual Command is very important, cause InstantCommands end right after they execute
            }
        };
    }

    @Override
    public void periodic() {
        //hardware.leftArm.setPosition(Math.min(armMiniTargetPosition, armTargetPosition));
        //hardware.rightArm.setPosition(Math.min(armMiniTargetPosition, armTargetPosition));
        OpModeReference.getInstance().getTelemetry().addData("Pitch Position", pitchingMotor.getCurrentPosition());
        OpModeReference.getInstance().getTelemetry().addData("Pitching RTP", runToPos);
        OpModeReference.getInstance().getTelemetry().addData("Pitching Power", pitchingMotor.getPower());
        isBusy = true;
    }

//    public RunCommand turnPitching() {
//        return new RunCommand(() -> {
//            isBusy = false;
//            if (globals.updateRobotStateTrue) {
//                if (stateToValueMap.containsKey(globals.getRobotState())) {
//                    if (globals.getRobotState() == RobotState.IDLE) {
//                        if (globals.lastRobotState == RobotState.INTAKE) {
//                            runToPos = stateToValueMap.get(globals.getRobotState());
//                        }
//                    } else {
//                        runToPos = stateToValueMap.get(globals.getRobotState());
//                    }
//                } else {
//                    runToPos = stateToValueMap.get(RobotState.IDLE);
//                }
//            }
//            pitchingMotor.setTargetPosition(runToPos);
//            pitchingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            //pitchingMotor.setPower(1);
//            pitchingMotor.setPower(gradualCalculatedPower());
//        }, this);
//    }}

    public InstantCommand adjustPitchingUp() {
        return new InstantCommand(()-> {
                    runToPos += 50;
                });

    }

    public InstantCommand adjustPitchingDown(){
        return new InstantCommand(()-> {
                    runToPos -= 50;
                });

    }

    private double gradualCalculatedPower() {
        double totalDistance = 830;
        double distanceLeft = globals.getRobotState() == RobotState.IDLE ? pitchingMotor.getCurrentPosition() : 830 - pitchingMotor.getCurrentPosition();
        return Math.min(Math.pow(((distanceLeft + 830*0.33)/830),8) + 0.15, 1);
    }

    public boolean isBusy() {
        if (!isBusy) {
            return pitchingMotor.isBusy();
        } else return true;
    }

}