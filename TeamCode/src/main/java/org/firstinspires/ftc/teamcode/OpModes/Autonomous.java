package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.AutoMode;
import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.HardwareRobot;

/**
 * Created by fibonacci on 1/12/18.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();



    static final float DRIVE_SPEED= 0.6f;
    static final float TURN_SPEED = 0.5f;
    static final float STRAFE_SPEED = 0.5f;
    static final float GRABBER_SPEED = 0.5f;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        if(robot.allianceColor != AllianceColor.NULL){
            switch(robot.autoMode){

                case IDOL_SIDE:
                    telemetry.addData("Auto Mode", "IDOL_SIDE");
                    telemetry.update();
                    //Drive to cryptobox
                    if(robot.allianceColor == AllianceColor.RED){
                        strafeForSeconds(1.25f, Direction.LEFT);
                    }else if(robot.allianceColor == AllianceColor.BLUE){
                        strafeForSeconds(1.25f, Direction.RIGHT);
                    }


                    //Push block to cryptobox
                    driveForSeconds(0.25f, -DRIVE_SPEED, -DRIVE_SPEED);
                    //Open Grabber
                    moveGrabberForSeconds(.75f, GRABBER_SPEED);
                    //Backup
                    driveForSeconds(0.1f, DRIVE_SPEED, DRIVE_SPEED);

                    //Push Glyph
                    driveForSeconds(0.25f, -DRIVE_SPEED, -DRIVE_SPEED);

                    //Backup
                    driveForSeconds(0.1f, DRIVE_SPEED, DRIVE_SPEED);

                    break;

                case CRYPTOBOX_SIDE:
                    telemetry.addData("Auto Mode", "CRYPTOBOX_SIDE");
                    telemetry.update();
                    //Drive to crytobox
                    driveForSeconds(0.5f, -DRIVE_SPEED, -DRIVE_SPEED);

                    //Strafe to Cryptobx Column
                    if(robot.allianceColor == AllianceColor.RED) {
                        strafeForSeconds(0.5f, Direction.LEFT);
                    }else if(robot.allianceColor == AllianceColor.BLUE){
                        strafeForSeconds(0.5f, Direction.RIGHT);
                    }

                    //Push block in
                    driveForSeconds(0.2f, -DRIVE_SPEED, -DRIVE_SPEED);

                    //Open Grabber
                    moveGrabberForSeconds(.75f, GRABBER_SPEED);
                    //Backup
                    driveForSeconds(0.1f, DRIVE_SPEED, DRIVE_SPEED);

                    //Push Glyph
                    driveForSeconds(0.25f, -DRIVE_SPEED, -DRIVE_SPEED);

                    //Backup
                    driveForSeconds(0.1f, DRIVE_SPEED, DRIVE_SPEED);
                    break;

                default:
                    System.out.println("Autonomous Mode does not exist!");


            }
        }else{
            telemetry.addData("Alliance Color Not Selected!", 0);
            telemetry.update();
        }
    }

    public void strafeForSeconds(float seconds, Direction direction){
        runtime.reset();
        if(direction == Direction.LEFT){
            robot.strafeLeft(STRAFE_SPEED);
        }else if(direction == Direction.RIGHT){
            robot.strafeRight(STRAFE_SPEED);
        }
        while (opModeIsActive() && (runtime.seconds() < seconds)){

        }
        robot.stopRobot();
        runtime.reset();

    }

    public void moveGrabberForSeconds(float seconds, float grabberPower){

        runtime.reset();
        robot.setGrabberPower(grabberPower);

        while (opModeIsActive() && (runtime.seconds() < seconds)){

        }
        robot.stopRobot();
        runtime.reset();
    }

    public void driveForSeconds(float seconds, float leftSpeed, float rightSpeed){

        runtime.reset();
        robot.driveAtSpeed(leftSpeed, rightSpeed);

        while (opModeIsActive() && (runtime.seconds() < seconds)){

        }
        robot.stopRobot();
        runtime.reset();
    }
}
