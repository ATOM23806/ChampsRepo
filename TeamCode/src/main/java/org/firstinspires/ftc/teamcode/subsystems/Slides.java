package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
public final class Slides {
    private DcMotor leftlift,rightlift;
    ArmBucket arm;

    public Slides(HardwareMap hw) {
        leftlift = hw.get(DcMotor.class, "leftlift");
        rightlift = hw.get(DcMotor.class, "rightlift");

        rightlift.setDirection(DcMotorSimple.Direction.REVERSE);
        arm = new ArmBucket(hw);
    }

    public void resetEnc() {
        leftlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftlift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightlift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void autoSlides(int target) {
        leftlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftlift.setPower(1);
        rightlift.setPower(1);
        leftlift.setTargetPosition(0);
        rightlift.setTargetPosition(0);

        leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftlift.setTargetPosition(target);
        rightlift.setTargetPosition(target);
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




    public void setZeroPower() {
        rightlift.setPower(0);
        leftlift.setPower(0);
    }



}
