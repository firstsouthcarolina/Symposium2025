package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class DriveConstants {
    public static final int kFrontLeftId = 1;
    public static final int kFrontRightId = 2;
    public static final int kBackLeftId = 3;
    public static final int kBackRightId = 4;

    public static final MotorType kMotorType = MotorType.kBrushed;

    public static final double kOpenLoopRampRateSeconds = 0.6;
}
