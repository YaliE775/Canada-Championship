package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@Config
public class DriveConstants {

        public static final String MOTOR_FR = "RF";
        public static final String MOTOR_LB = "LR";
        public static final String MOTOR_FL = "LF";
        public static final String MOTOR_RB = "RR";

        public static double KP = 0.01;
        public static double KI = 0;
        public static double KD = 0;

        public static int TOLERANCE = 5;
        public static int MAX_SET_POINT_COUNT = 50;

        public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;

        public static RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT ;

        public static int WALL_ROTATE = -90;
        public static int INTAKE_FROM_OBSERVATION = 180;
        public static int START_AUTO_AGAIN = -180;

}