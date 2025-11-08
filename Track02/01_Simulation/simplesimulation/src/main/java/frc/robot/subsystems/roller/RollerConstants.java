package frc.robot.subsystems.roller;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;

public class RollerConstants {
    /*
     * Change this to what the real robot has
     */
    public static final int kCanID = 31; 
    public static final MotorType kMotorType = MotorType.kBrushed;
    public static final DCMotor kMotor = DCMotor.getNEO(1);

    public static final int kUpdatePeriodMilliseconds = 20;
    public static final double kMotorReduction = 1.0;
    public static final double kMoi = 1.0;
}