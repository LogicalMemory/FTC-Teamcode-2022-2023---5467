package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoBase;

@Autonomous(name = "Auto", group = "autonomous")
public class autoLeft extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initRobo();

        telemetry.addLine("Robo Init Good");
        telemetry.update();

        waitForStart();
        telemetry.addLine("Robo Started");
        telemetry.update();

        move(1, 0, 0, 0.5);

        sleep(1000);

        move(0, 1, 0, 0.5);



        // y pos = forward,
        // y neg = backward,
        // x pos = right,
        // x neg = left

        //vtur turns


        sleep(1000);

        setSinglePow(0);

    }
}
