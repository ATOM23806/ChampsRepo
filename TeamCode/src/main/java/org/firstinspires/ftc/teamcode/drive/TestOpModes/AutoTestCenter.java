package org.firstinspires.ftc.teamcode.drive.TestOpModes;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.camera.Tram;
import org.firstinspires.ftc.teamcode.drive.mechanisms.Slides;
import org.firstinspires.ftc.vision.VisionPortal;

public class AutoTestCenter extends LinearOpMode {
    private Servo rights, lefts;
    private CRServo intake;

    private VisionPortal visionPortal;
    private Tram redTram;
    private RevColorSensorV3 colorSensorV3;
    private Servo release;
    Slides slides;

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
