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

    double Varx = 0;
    double Vary = 0;
    double qx = 0;
    double qy = 0;
    double Kx = 0;
    double Ky = 0;
    double r = 0;

    long currenttime = 0;
    long previoustime = 0; // better way to do time?

    double ReqAngle = 0;

    ElapsedTime Timer = new ElapsedTime();

    public KalmanFilter() {
        setDefaultCommand(new PerpetualCommand(calculateAngle()));
        Timer.reset();
    }

    public RunCommand calculateAngle(){
        return new RunCommand(()->{
            //Position estimation

            previoustime=currenttime;
            currenttime=Timer.nanoseconds();
            prevx = x;
            prevy = y;
            //x = x+(velocity*(currenttime-previoustime));
            //y = y+(velocity*(currenttime-previoustime));


            // Update Variance
            qx = Varx * ((currenttime-previoustime)*(currenttime-previoustime));
            Varx = Varx + qx;

            qy = Vary * ((currenttime-previoustime)*(currenttime-previoustime));
            Vary = Vary + qy;

            //Calculate Kalman Gain
            Kx=(Varx/(Varx+r));
            Ky=(Vary/(Vary+r));

            //x = prevx + Kx*(Measurementx-x);
            //y = prevy + Ky*(Measurementy-y);

            //Update Variance

            Varx *= (1-Kx);
            Vary *= (1-Ky);

            // Calculate angle
            ReqAngle = Math.atan(x/y);
        }, this);
    }
}
