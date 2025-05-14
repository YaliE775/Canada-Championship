package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class Teleop extends CommandOpMode {
        private GamepadEx driverController;

        private Drive drive;


        @Override
        public void initialize() {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            drive = new Drive(hardwareMap, telemetry);

            driverController = new GamepadEx(gamepad1);

            drive.setDefaultCommand(
                    drive.driveFieldCentric(
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

