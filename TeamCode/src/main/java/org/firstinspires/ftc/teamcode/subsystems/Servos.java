package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Constants.ServoConstants;

public class Servos {

    private final Servo right, left, release, plane;
    private final CRServo intake;

    private static volatile Servos instance = null;

    public static Servos getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            synchronized (Servos.class) {
                if (instance == null) {
                    instance = new Servos(hardwareMap);
                }
            }
        }
        return instance;
    }

    private Servos(HardwareMap hardwareMap) {
        right = hardwareMap.get(Servo.class, ServoConstants.RIGHT_ARM);
        left = hardwareMap.get(Servo.class, ServoConstants.LEFT_ARM);
        intake = hardwareMap.get(CRServo.class, ServoConstants.INTAKE);
        release = hardwareMap.get(Servo.class, ServoConstants.RELEASE);
        plane = hardwareMap.get(Servo.class, ServoConstants.PLANE);

    }

    public void init() {
        setPivot(0);
        setRelease(0);
    }

    public void setPivot(double pos) {
        right.setPosition(pos);
        left.setPosition(pos);
    }

    public void setIntake(double speed) {
        intake.setPower(speed);
    }

    public void setRelease(double pos) {
        release.setPosition(0);
    }
}
