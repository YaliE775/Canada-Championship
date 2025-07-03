package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.Drive;

public class Teleop extends CommandOpMode {

    private GamepadEx driverController;


    @Override
        public void initialize() {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Drive drive = new Drive(hardwareMap, telemetry);

            driverController = new GamepadEx(gamepad1);

            drive.setDefaultCommand(
                    drive.driveRobotCentric(
                            ()-> -driverController.getLeftY(),
                            ()-> -driverController.getLeftX(),
                            ()-> -driverController.getRightX())
            );

            driverController.getGamepadButton(GamepadKeys.Button.B).whenActive(drive.setDriveAngle(90));
            driverController.getGamepadButton(GamepadKeys.Button.A).whenActive(drive.resetIMU());

        }

        @Override
        public void run() {
            super.run();
            telemetry.update();
        }
    }
