package org.firstinspires.ftc.teamcode.testteleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Joint;

@TeleOp
public class JointTeleop extends CommandOpMode {

    private GamepadEx controller;
        
    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Joint joint = new Joint(telemetry, hardwareMap);

        controller = new GamepadEx(gamepad1);
        controller.getGamepadButton(GamepadKeys.Button.A).whenPressed(joint.setSetPoint(Constants.ShoulderConstants.DEFAULT_ANGLE));
        controller.getGamepadButton(GamepadKeys.Button.B).whenPressed(joint.setSetPoint(Constants.ShoulderConstants.PARALLEL_ANGLE));
        controller.getGamepadButton(GamepadKeys.Button.X).whenPressed(joint.setSetPoint(Constants.ShoulderConstants.SCORE_SPECIMEN_HIGH));
        controller.getGamepadButton(GamepadKeys.Button.Y).whenPressed(joint.setSetPoint(Constants.ShoulderConstants.SCORE_ANGLE_BASKET));

    }

    @Override
    public void run() {
        super.run();
        telemetry.update();

   }
}