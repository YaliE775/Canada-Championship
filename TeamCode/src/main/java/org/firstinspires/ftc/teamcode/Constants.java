package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
public class Constants {
    @Config
    public static class TelescopeConstants{
        public static final String ELEVATOR_MOTOR = "Lift";

        public static double KP = 0.0045;
        public static double KI = 0.00000005;
        public static double KD = 0.00000001;
        public static double TICK_TO_CM = 0.020444444;
        public static double DEFAULT_POSITION = 200;
        public static double LOW_BASKET = 470;
        public static double HIGH_BASKET = 2420;
        public static double PRE_SCORE_HIGH_SPECIMEN = 1675;
        public static double SCORE_LOW_SPECIMEN = 0;
        public static double TOLERANCE = 15;
    }
    @Config
    public static class ShoulderConstants {
        public static final String MASTER_MOTOR = "Master";
        public static final String SLAVE_MOTOR = "Slave";

        public static double KP = 0.0015;
        public static double KI = 0.002;
        public static double KD = 0.00005;
        public static double kS = 0;
        public static double kG = 0.22;

        public static double MAX_VELOCITY = 20000;
        public static double ACCELERATION = 21000;
        public static double ACCELERATION_HIGH = 10000;
        public static double PARALLEL_ANGLE = 800;

        public static double SCORE_ANGLE_BASKET = 2500;
        public static double DEFAULT_ANGLE = 200;
        public static double ROBOT_START_ANGLE = 500;
        public static double SCORE_SPECIMEN_HIGH = 2000;

        public static double TOLERANCE = 25;
        public static double ONE_TICK = 0.044;
        public static double STARTING_ANGLE = 70;
    }

}
