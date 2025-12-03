package org.firstinspires.ftc.teamcode.common.util;

import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.common.subsystems.Limelight;

public class ShooterCalcs extends SubsystemBase {
    public double a = 0;
    public double b = 0;
    public double c = 0;
    public double d = 0;
    public double hoodAngleReq = 0;
    public double angleConverter = 0;
    public double speedConverter = 0;
    public double TrueSpeed = 0;
    //ax+by+cz=d

    public ShooterCalcs(){
        setDefaultCommand(new PerpetualCommand(calculateShooting()));
    }

    public RunCommand calculateShooting(){
        return new RunCommand(()->{
           /* TrueSpeed = OpModeReference.getInstance().outtakeSubSystem.flywheel.getVelocity()*speedConverter;
            hoodAngleReq = angleConverter*((d-a*(OpModeReference.getInstance().limelightSubsystem.distance -c*TrueSpeed))/b);*/
        },this);
    }
}
