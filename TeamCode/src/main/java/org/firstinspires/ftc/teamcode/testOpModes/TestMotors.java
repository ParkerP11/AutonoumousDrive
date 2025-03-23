package org.firstinspires.ftc.teamcode.testOpModes;

import static org.firstinspires.ftc.teamcode.AutonomousDrive.setTimeLimit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousDrive;

@Autonomous(name = "Motor Tester")
public class TestMotors extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        AutonomousDrive ad = new AutonomousDrive(this, 1);
        setTimeLimit(4);
        ad.setOutputInfo(true);
        waitForStart();
        for(int i = 0; i < 4; i++){
            int speed = 0;
            while(speed < 100) {
                ad.getMotor(i).setPower(speed/100);
                sleep(10);
                speed++;
            }
            double startTime = time;
            while (time - startTime < 2){
                ad.getMotor(i).setPower(1);
            }
            while (speed > -100){
                ad.getMotor(i).setPower(speed/100);
                sleep(5);
                speed--;
            }
            startTime = time;
            while (time - startTime < 2){
                ad.getMotor(i).setPower(-1);
            }
            while (speed < 0){
                ad.getMotor(i).setPower(speed/100);
                sleep(2);
                speed++;
            }
        }

    }
}
