package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoBase;

@Autonomous(name = "AutoRight", group = "autonomous")
public class autoRight extends AutoBase {

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



        if (signal == 3) {
            move(0, -1, 0, 0.3);
            sleep(1510*2);

        } else if (signal == 2) {
            move(0, -1, 0, 0.3);
            sleep(1510);

        } else if (signal == 1) {

        } else {

        }


        setSinglePow(0);
        // y pos = forward,
        // y neg = backward,
        // x pos = right,
        // x neg = left

        //vtur turns



    }
}
