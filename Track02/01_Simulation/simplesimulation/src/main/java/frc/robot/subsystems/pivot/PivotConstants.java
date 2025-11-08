package frc.robot.subsystems.pivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class PivotConstants {
    public static final DCMotor dcMotor = DCMotor.getNEO(1);
    public static final int canID = 1;
    public static final double gearRatio = 15;
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0.1;
    public static final double maxVelocity = 1; // rad/s
    public static final double maxAcceleration = 10; // rad/s²
    public static final boolean brakeMode = true;
    public static final boolean enableStatorLimit = true;
    public static final int statorCurrentLimit = 40;
    public static final boolean enableSupplyLimit = false;
    public static final double supplyCurrentLimit = 40;

    public static final double minimumAngleRad = Units.degreesToRadians(-90);
    public static final double maximumAngleRad = Units.degreesToRadians(90);
    public static final double startingAngleRad = Units.degreesToRadians(0);
    public static final double armLengthMeters = 0.1;
    public static final double moi = 0.01; // moment of inertia
    public static final boolean simulateGravity = false;
}
