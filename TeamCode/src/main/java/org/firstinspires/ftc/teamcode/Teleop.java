package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class Teleop extends CommandOpMode{

    @Override
    public void initialize() {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            GamepadEx controller = new GamepadEx(gamepad1);

            IntakeJoint intakeJoint = new IntakeJoint(hardwareMap,telemetry);

            intakeJoint.setDefaultCommand(intakeJoint.defaultCommand());

            controller.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileActiveContinuous(intakeJoint.defaultCommand());
            controller.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileActiveContinuous(intakeJoint.intakeSubmersible());

    }
}
