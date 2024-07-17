package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Constants.SlideConstants;


public class Slides {

    private final DcMotorEx leftlift, rightlift;

    private static volatile Slides instance = null;

    public static Slides getInstance(HardwareMap map) {
        if (instance == null) {
            //TODO try just "this" as well
            synchronized (Slides.class) {
                if (instance == null) {
                    instance = new Slides(map);
                }
            }
        }
        return instance;
    }

    private Slides(HardwareMap hw) {
        leftlift = hw.get(DcMotorEx.class, SlideConstants.LEFT_LIFT);
        rightlift = hw.get(DcMotorEx.class, SlideConstants.RIGHT_LIFT);

        rightlift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void resetEnc() {
        leftlift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightlift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftlift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightlift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    int lastError = 0;
    double Kp = 0.004;
    double Ki = 0;
    double Kd = 0;
    public boolean PIDMotion(int reference) {


        int ref = reference;
        double integralSum = 0;
        int error = rightlift.getCurrentPosition() - reference;
        ElapsedTime timer = new ElapsedTime();

        if(Math.abs(error) > 20) {
            double derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            rightlift.setPower(out);
            leftlift.setPower(out);
            lastError = error;
            timer.reset();
        }
        return Math.abs(error) > 20;
    }


    public void stop() {
        rightlift.setPower(0);
        leftlift.setPower(0);
    }
}
