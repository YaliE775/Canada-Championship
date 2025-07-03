package org.firstinspires.ftc.teamcode.testteleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.sybsystems.Telescope;

@TeleOp
public class TelescopeTeleop extends CommandOpMode {
        @Override
        public void initialize() {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            GamepadEx controller = new GamepadEx(gamepad1);
            Telescope telescope = new Telescope(telemetry, hardwareMap);

            telescope.setDefaultCommand(telescope.defaultCommand());

            controller.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileActiveContinuous(telescope.setSetPoint(Constants.TelescopeConstants.LOW_BASKET));
            controller.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileActiveContinuous(telescope.setSetPoint(Constants.TelescopeConstants.HIGH_BASKET));
            controller.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whileActiveContinuous(telescope.setSetPoint(Constants.TelescopeConstants.SCORE_LOW_SPECIMEN));
            controller.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whileActiveContinuous(telescope.setSetPoint(Constants.TelescopeConstants.PRE_SCORE_HIGH_SPECIMEN));
        }
    }

