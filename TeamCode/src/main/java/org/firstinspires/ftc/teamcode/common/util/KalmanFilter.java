package org.firstinspires.ftc.teamcode.common.util;

import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.util.Timing;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.NanoTimer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.common.subsystems.PedroPathing;

public class KalmanFilter extends SubsystemBase {
    public static final KalmanFilter INSTANCE = new KalmanFilter();
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
    double r = 0.0001;

    long currenttime = 0;
    long previoustime = 0; // better way to do time?

    double ReqAngle = 0;

    ElapsedTime Timer;

    public KalmanFilter() {
        Timer = new ElapsedTime();
        Timer.reset();
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

            //Calculate Kalman Gain
            Kx=(Varx/(Varx+r));
            Ky=(Vary/(Vary+r));

            x = prevx + Kx*(OpModeReference.getInstance().pedroPathing.getX()-prevx);
            y = prevy + Ky*(OpModeReference.getInstance().pedroPathing.getY()-prevy);

            //Update Variance

            Varx *= (1-Kx);
            Vary *= (1-Ky);

            // Calculate angle
            if (OpModeReference.getInstance().isRedAlliance) {
                ReqAngle = Math.toDegrees(Math.atan((144 - x) / (144 - y)));
            } else{
                ReqAngle = Math.toDegrees(Math.atan((-x) / (144 - y)));
            }
        }, this);
    }

    @Override
    public void periodic(){
        OpModeReference.getInstance().getTelemetry().addData("Angle to Goal",ReqAngle);
        OpModeReference.getInstance().getTelemetry().addData("x Kalman",x);
        OpModeReference.getInstance().getTelemetry().addData("y Kalman",y);
    }
}
