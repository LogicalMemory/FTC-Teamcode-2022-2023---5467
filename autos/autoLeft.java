package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoBase;

@Autonomous(name = "AutoLeft", group = "autonomous")
public class autoLeft extends AutoBase {

    //Signal on left is flipped on right, for left it starts on 3, mid is 2, farthest is 1
    public int signal = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobo();

        telemetry.addLine("Robo Init Good");
        telemetry.update();

        waitForStart();
        telemetry.addLine("Robo Started");
        telemetry.update();



        //Center of starting Tile, move 4.5in
        move(0, -1, 0, 0.3);
        sleep(200);



        if (signal == 1) {
            move(0, -1, 0, 0.3);
            sleep(1510*2);

        } else if (signal == 2) {
            move(0, -1, 0, 0.3);
            sleep(1510);

        } else if (signal == 3) {

        } else {

        }

        setSinglePow(0);

        // y neg = forward,
        // y pos = backward,
        // x pos = right,
        // x neg = left

        //vtur turns


        //Conversions
        //10.5in at .3sp for 1sec right
        //15.7in at .3sp for 1sec backwards

        //each tile = 23.7in
        //each tile = 1.510sec


        //15.7/1
        //4.5/x
        //distDesired/15.7 = time


    }
}
