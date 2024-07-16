package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public final class ArmBucket {

    private Servo left,right,release;

    public ArmBucket(HardwareMap hardwareMap) {
        right = hardwareMap.get(Servo.class, "rA");
        left = hardwareMap.get(Servo.class, "lA");
        release = hardwareMap.get(Servo.class, "release");
    }

    public void releaseOpen() {
        release.setPosition(1);
        left.setPosition(0.5);
        try{
            Thread.sleep(100);
        } catch (InterruptedException e){};
        left.setPosition(0);
    }

    public void readyScore() {
        release.setPosition(0);
        left.setPosition(0.5);
    }

    public void scorePos() {
        left.setPosition(0.35);
    }
    public void closePos() {
        left.setPosition(0.8);
    }


}
