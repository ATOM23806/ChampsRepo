package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Servos;

public class ToiletAnanasNasdas extends OpMode {

    private Servos servos;
    @Override
    public void init() {
        servos = Servos.getInstance(hardwareMap);
        servos.init();

    }

    @Override
    public void loop() {

    }


}
