package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.CompVision;

@Autonomous(name = "CamTest", group = "autonomous")
public class CamTest extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        cam = new CompVision(hardwareMap, 1);
        waitForStart();
    }
}
