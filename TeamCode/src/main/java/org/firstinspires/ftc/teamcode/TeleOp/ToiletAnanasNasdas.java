package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Servos;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

public class ToiletAnanasNasdas extends OpMode {

    private Servos servos;
    private Slides slides;
    @Override
    public void init() {
        servos = Servos.getInstance(hardwareMap);
        slides = Slides.getInstance(hardwareMap);

        servos.init();
        slides.resetEnc();

    }

    @Override
    public void loop() {

    }


}
