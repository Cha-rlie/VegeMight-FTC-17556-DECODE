package org.firstinspires.ftc.teamcode.common.util;

import android.graphics.Path;

import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.OpModeReference;

public class VegeKalmanFilter extends SubsystemBase {
    //public static final VegeKalmanFilter INSTANCE = new VegeKalmanFilter();
    double x = 0;  //(get value from pinpoint/autonomous) - starting value
    double prevx = 0;
    double y = 0; //(get value from pinpoint/autonomous) - starting value
    double prevy = 0;

    double Varx = 0.00001;
    double Vary = 0.00001;
    double qx = 0;
    double qy = 0;
    double Kx = 0;
    double Ky = 0;
    double rPinpoint = 0.0001;
    double rLimelight = 0.001;

    long currenttime = 0;
    long previoustime = 0; // better way to do time?

    public double ReqAngle = 0; // Degrees
    public double ReqDist = 0;

    ElapsedTime Timer;
    GoBildaPinpointDriver pinpointDriver;

    public VegeKalmanFilter() {
        Timer = new ElapsedTime();
        Timer.reset();
        pinpointDriver = OpModeReference.getInstance().getHardwareMap().get(GoBildaPinpointDriver.class,"pinpoint");
        setDefaultCommand(new PerpetualCommand(calculateAngle()));
    }

    public RunCommand calculateAngle(){
        return new RunCommand(()->{
            //Position estimation

            previoustime=currenttime;
            currenttime=Timer.nanoseconds();
            prevx = x;
            prevy = y;
            x = x+(OpModeReference.getInstance().pedroPathing.getVelocityX()*(currenttime-previoustime));
            y = y+(OpModeReference.getInstance().pedroPathing.getVelocityY()*(currenttime-previoustime));


            // Update Variance
            qx = Varx * ((currenttime-previoustime)*(currenttime-previoustime));
            Varx = Varx + qx;

            qy = Vary * ((currenttime-previoustime)*(currenttime-previoustime));
            Vary = Vary + qy;

            Kx = (Varx / (Varx + rPinpoint));
            Ky = (Vary / (Vary + rPinpoint));


            x = prevx + Kx * (OpModeReference.getInstance().pedroPathing.getX() - prevx);
            y = prevy + Ky * (OpModeReference.getInstance().pedroPathing.getY() - prevy);

            //Update Variance

            Varx *= (1 - Kx);
            Vary *= (1 - Ky);

            // Calculate angle
            if (OpModeReference.getInstance().isRedAlliance) {
                ReqAngle = Math.toDegrees(Math.atan2((144 - y),(144 - x)));
                ReqDist = Math.sqrt((144-x)*(144-x)+(144-y)*(144-y));
            } else {
                ReqAngle = Math.toDegrees(Math.atan2((144-y),(- x)));
                ReqDist = Math.sqrt((-x)*(-x)+(144-y)*(144-y));
            }

            /*if (pinpointDriver.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
                //Calculate Kalman Gain
            }*/
            /*if (OpModeReference.getInstance().limelightSubsystem.resultIsValid) {
                //Calculate Kalman Gain
                Kx = (Varx / (Varx + rLimelight));
                Ky = (Vary / (Vary + rLimelight));


                x = prevx + Kx * (OpModeReference.getInstance().limelightSubsystem. - prevx);
                y = prevy + Ky * (OpModeReference.getInstance().limelightSubsystem. - prevy);

                //Update Variance

                Varx *= (1 - Kx);
                Vary *= (1 - Ky);

                // Calculate angle
                if (OpModeReference.getInstance().isRedAlliance) {
                    ReqAngle = Math.toDegrees(Math.atan((144 - x) / (144 - y)));
                } else {
                    ReqAngle = Math.toDegrees(Math.atan((-x) / (144 - y)));
                }
            }*/
        }, this);
    }

    @Override
    public void periodic(){
        OpModeReference.getInstance().getTelemetry().addData("Angle to Goal",ReqAngle);
        OpModeReference.getInstance().getTelemetry().addData("x Kalman",x);
        OpModeReference.getInstance().getTelemetry().addData("y Kalman",y);
    }

    public void reset(Pose newPose) {
        x = newPose.getX();
        y = newPose.getY();
        Varx = 0.00001;
        Vary = 0.00001;
    }
}
