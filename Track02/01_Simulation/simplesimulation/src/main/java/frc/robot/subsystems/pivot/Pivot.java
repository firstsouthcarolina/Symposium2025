package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ControlMode;

/**
 * Pivot subsystem using SparkMAX with NEO motor
 * 
 * Partially generated with YAMG
 */
@Logged(name = "Pivot")
public class Pivot extends SubsystemBase {
    // Feedforward
    private final ArmFeedforward feedforward = new ArmFeedforward(
            PivotConstants.kS, // kS
            0, // kG - Pivot doesn't need gravity compensation
            PivotConstants.kV, // kV
            PivotConstants.kA // kA
    );

    // Motor controller
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkSim motorSim;

    private ControlMode currentControlMode = ControlMode.OPEN_LOOP;
    private double targetPositionRotations = 0.0;
    private double targetVelocity = 0.0;

    // Profiled PID Controller
    private ProfiledPIDController profiledPIDController;
    private TrapezoidProfile.Constraints constraints;
    private Notifier controlLoop;
    private final double CONTROL_LOOP_FREQUENCY = 100.0; // Hz

    // Simulation
    private final SingleJointedArmSim pivotSim;

    /**
     * Creates a new Pivot Subsystem.
     */
    public Pivot() {
        // Initialize motor controller
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motor = new SparkMax(PivotConstants.canID, MotorType.kBrushless);
        motorConfig.idleMode(PivotConstants.brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

        // Configure encoder
        encoder = motor.getEncoder();
        encoder.setPosition(0);

        // Set current limits
        motorConfig.smartCurrentLimit(PivotConstants.statorCurrentLimit);

        // Save configuration
        motor.configure(
                motorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        motorSim = new SparkSim(motor, PivotConstants.dcMotor);

        // Initialize simulation
        pivotSim = new SingleJointedArmSim(
                PivotConstants.dcMotor, // Motor type
                PivotConstants.gearRatio,
                PivotConstants.moi, // Arm moment of inertia - Small value since there are no arm parameters
                PivotConstants.armLengthMeters, // Arm length (m) - Small value since there are no arm parameters
                PivotConstants.minimumAngleRad, // Min angle (rad)
                PivotConstants.maximumAngleRad, // Max angle (rad)
                PivotConstants.simulateGravity, // Simulate gravity - Disable gravity for pivot
                PivotConstants.startingAngleRad // Starting position (rad)
        );

        // Initialize ProfiledPIDController
        // Convert from radians to rotations for constraints
        double maxVelocityRotations = PivotConstants.maxVelocity / (2.0 * Math.PI);
        double maxAccelerationRotations = PivotConstants.maxAcceleration / (2.0 * Math.PI);

        constraints = new TrapezoidProfile.Constraints(
                maxVelocityRotations,
                maxAccelerationRotations);
        profiledPIDController = new ProfiledPIDController(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD,
                constraints);

        // Start control loop on a separate thread using Notifier
        controlLoop = new Notifier(this::controlLoopFn);
        controlLoop.startPeriodic(1.0 / CONTROL_LOOP_FREQUENCY);
    }

    /**
     * Control loop function that runs at a fixed frequency.
     * This is used for SparkMAX and SparkFlex controllers to implement
     * closed-loop control outside of the main robot loop.
     */
    private void controlLoopFn() {
        switch (currentControlMode) {
            case POSITION:
                double currentPositionRotations = getPositionRotations();
                double output = profiledPIDController.calculate(
                        currentPositionRotations,
                        targetPositionRotations);
                double velocity = profiledPIDController.getSetpoint().velocity;
                double feedforwardOutput = feedforward.calculate(0, velocity); // No gravity compensation for pivot
                setVoltage(output + feedforwardOutput);
                break;
            case VELOCITY:
                double currentVel = getVelocity();
                double velOutput = profiledPIDController.calculate(
                        currentVel,
                        targetVelocity);
                double accel = profiledPIDController.getSetpoint().velocity - currentVel;
                double velFeedforwardOutput = feedforward.calculate(
                        0,
                        targetVelocity,
                        accel); // No gravity compensation

                // Apply the combined PID output and feedforward to the motor
                double velocityVoltage = velOutput + velFeedforwardOutput;
                motor.setVoltage(velocityVoltage);
                break;
            case OPEN_LOOP:
            default:
                // Do nothing, voltage is set directly
                break;
        }
    }

    /**
     * Update simulation and telemetry.
     */
    @Override
    public void periodic() {
    }

    /**
     * Update simulation.
     */
    @Override
    public void simulationPeriodic() {
        // Set input voltage from motor controller to simulation
        // Note: This may need to be talonfx.getSimState().getMotorVoltage() as the
        // input
        // pivotSim.setInput(dcMotor.getVoltage(dcMotor.getTorque(pivotSim.getCurrentDrawAmps()),
        // pivotSim.getVelocityRadPerSec()));
        // pivotSim.setInput(getVoltage());
        // Set input voltage from motor controller to simulation
        // Use getVoltage() for other controllers
        pivotSim.setInput(getVoltage());

        // Update simulation by 20ms
        pivotSim.update(0.020);
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(
                        pivotSim.getCurrentDrawAmps()));

        double motorPosition = Radians.of(pivotSim.getAngleRads() * PivotConstants.gearRatio).in(
                Rotations);
        double motorVelocity = RadiansPerSecond.of(
                pivotSim.getVelocityRadPerSec() * PivotConstants.gearRatio).in(RotationsPerSecond);
        motorSim.iterate(motorVelocity * 60, RoboRioSim.getVInVoltage(), 0.02);
    }

    /**
     * Get the current position in Rotations.
     * 
     * @return Position in Rotations
     */
    @Logged(name = "Position/Rotations")
    public double getPositionRotations() {
        // Rotations
        return encoder.getPosition() / PivotConstants.gearRatio;
    }

    @Logged(name = "Position/Radians")
    public double getPositionRadians() {
        return Radians.convertFrom(getPositionRotations(), Rotations);
    }

    /**
     * Get the current velocity in rotations per second.
     * 
     * @return Velocity in rotations per second
     */
    @Logged(name = "Velocity")
    public double getVelocity() {
        return encoder.getVelocity() / PivotConstants.gearRatio / 60.0; // Convert from RPM to RPS
    }

    /**
     * Get the current applied voltage.
     * 
     * @return Applied voltage
     */
    @Logged(name = "Voltage")
    public double getVoltage() {
        return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    /**
     * Get the current motor current.
     * 
     * @return Motor current in amps
     */
    public double getCurrent() {
        return motor.getOutputCurrent();
    }

    /**
     * Get the current motor temperature.
     * 
     * @return Motor temperature in Celsius
     */
    public double getTemperature() {
        return motor.getMotorTemperature();
    }

    /**
     * Set pivot angle.
     * 
     * @param angleDegrees The target angle in degrees
     */
    public void setAngle(double angleDegrees) {
        setAngle(angleDegrees, 0);
    }

    /**
     * Set pivot angle with acceleration.
     * 
     * @param angleDegrees The target angle in degrees
     * @param acceleration The acceleration in rad/s²
     */
    public void setAngle(double angleDegrees, double acceleration) {
        // Convert degrees to rotations
        double positionRotations = Units.degreesToRotations(angleDegrees);
        // double positionRotations = angleRadians / (2.0 * Math.PI);

        // Use the ProfiledPIDController
        targetPositionRotations = positionRotations;
        currentControlMode = ControlMode.POSITION;

        // If acceleration is specified, update constraints
        if (acceleration > 0) {
            double maxAccelRotations = acceleration / (2.0 * Math.PI);
            constraints = new TrapezoidProfile.Constraints(
                    constraints.maxVelocity,
                    maxAccelRotations);
            profiledPIDController.setConstraints(constraints);
        }
    }

    /**
     * Set pivot angular velocity.
     * 
     * @param velocityDegPerSec The target velocity in degrees per second
     */
    public void setVelocity(double velocityDegPerSec) {
        setVelocity(velocityDegPerSec, 0);
    }

    /**
     * Set pivot angular velocity with acceleration.
     * 
     * @param velocityDegPerSec The target velocity in degrees per second
     * @param acceleration      The acceleration in degrees per second squared
     */
    public void setVelocity(double velocityDegPerSec, double acceleration) {
        // Convert degrees/sec to rotations/sec
        double velocityRadPerSec = Units.degreesToRadians(velocityDegPerSec);
        double velocityRotations = velocityRadPerSec / (2.0 * Math.PI);

        // Use the ProfiledPIDController
        targetVelocity = velocityRotations;
        currentControlMode = ControlMode.VELOCITY;

        // If acceleration is specified, update constraints
        if (acceleration > 0) {
            double maxAccelRotations = Units.degreesToRadians(acceleration) / (2.0 * Math.PI);
            constraints = new TrapezoidProfile.Constraints(
                    constraints.maxVelocity,
                    maxAccelRotations);
            profiledPIDController.setConstraints(constraints);
        }

        // Apply velocity directly to the motor controller as well
        // This ensures immediate response while the control loop refines it
        double ffVolts = feedforward.calculate(0, velocityRotations, 0); // No gravity compensation for pivot
        motor.setVoltage(ffVolts);
    }

    /**
     * Set motor voltage directly.
     * 
     * @param voltage The voltage to apply
     */
    public void setVoltage(double voltage) {
        currentControlMode = ControlMode.OPEN_LOOP;
        motor.setVoltage(voltage);
    }

    /**
     * Get the pivot simulation for testing.
     * 
     * @return The pivot simulation model
     */
    public SingleJointedArmSim getSimulation() {
        return pivotSim;
    }

    /**
     * Creates a command to set the pivot to a specific angle.
     * 
     * @param angleDegrees The target angle in degrees
     * @return A command that sets the pivot to the specified angle
     */
    public Command setAngleCommand(double angleDegrees) {
        return runOnce(() -> setAngle(angleDegrees));
    }

    
    public Command setAngleCommand(double angleDegrees, double accelerationRadsPerSec) {
        return runOnce(() -> setAngle(angleDegrees, accelerationRadsPerSec));
    }

    /**
     * Creates a command to move the pivot to a specific angle with a profile.
     * 
     * @param angleDegrees The target angle in degrees
     * @return A command that moves the pivot to the specified angle
     */
    public Command moveToAngleCommand(double angleDegrees) {
        return run(() -> {
            // Just set the position and let the profiled controller handle it
            setAngle(angleDegrees);
        })
                .until(() -> {
                    double currentAngle = Units.radiansToDegrees(getPositionRadians());
                    return Math.abs(angleDegrees - currentAngle) < 2.0; // 2 degree tolerance
                })
                .finallyDo(interrupted -> setVelocity(0));
    }

    /**
     * Creates a command to stop the pivot.
     * 
     * @return A command that stops the pivot
     */
    public Command stopCommand() {
        return runOnce(() -> setVelocity(0));
    }

    /**
     * Creates a command to move the pivot at a specific velocity.
     * 
     * @param velocityDegPerSec The target velocity in degrees per second
     * @return A command that moves the pivot at the specified velocity
     */
    public Command moveAtVelocityCommand(double velocityDegPerSec) {
        return run(() -> setVelocity(velocityDegPerSec));
    }
}
