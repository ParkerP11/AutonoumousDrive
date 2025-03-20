package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousDrive;
import static org.firstinspires.ftc.teamcode.AutonomousDrive.*;


@Autonomous(name = "PID TUNER")
public class TestPID extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        AutonomousDrive ad = new AutonomousDrive(this, 1);
        setTimeLimit(4);
        ad.setOutputInfo(true);
        for(int i = 1; i < 10; i++){
            for(int k = 1; k < 10; k++){
                for (int h = 2; h < 24; h++){
                    ad.setPID(h/2, k/10, k/10, 0);
                    ad.goToPointConstantHeading(30, 24);
                    sleep(1000);
                    ad.goToPointConstantHeading(8, 36);
                }
            }
        }


    }
}
