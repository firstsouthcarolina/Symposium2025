package frc.robot.subsystems.roller;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SparkUtil;

// Log at the Subsystem level
@Logged(name = "Roller")
public class Roller extends SubsystemBase {
    private final SparkMax rollerMotor;
    private final RelativeEncoder rollerEncoder;

    // Only needed for Simulation
    private final DCMotorSim motorSim;

    public Roller() {
        rollerMotor = new SparkMax(RollerConstants.kCanID, RollerConstants.kMotorType);
        rollerEncoder = rollerMotor.getEncoder();

        if (RobotBase.isSimulation()) {
            motorSim = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(
                            RollerConstants.kMotor,
                            RollerConstants.kMoi,
                            RollerConstants.kMotorReduction),
                    RollerConstants.kMotor);
        } else {
            motorSim = null;
        }
        configureMotorSettings();
    }

    @Override
    public void simulationPeriodic() {
        motorSim.setInputVoltage(getVoltage());
        motorSim.update(Constants.dtSeconds);

        SmartDashboard.putNumber(getName() + "/Voltage", getVoltage());
        SmartDashboard.putNumber(getName() + "/Velocity", getVelocity());
        SmartDashboard.putNumber(getName() + "/Current", getOutputCurrent());

        // Simulate power draw over time
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(
                        motorSim.getCurrentDrawAmps()));

    }

    /**
     * Configures motor settings for coral motor, can be tweaked for some tuning if
     * needed (Needed)
     */
    private void configureMotorSettings() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0);
        config.encoder
                .uvwMeasurementPeriod(RollerConstants.kUpdatePeriodMilliseconds)
                .uvwAverageDepth(2);
        config.signals
                .primaryEncoderPositionPeriodMs(RollerConstants.kUpdatePeriodMilliseconds)
                .primaryEncoderVelocityPeriodMs(RollerConstants.kUpdatePeriodMilliseconds)
                .appliedOutputPeriodMs(RollerConstants.kUpdatePeriodMilliseconds)
                .busVoltagePeriodMs(RollerConstants.kUpdatePeriodMilliseconds)
                .outputCurrentPeriodMs(RollerConstants.kUpdatePeriodMilliseconds);
        SparkUtil.tryUntilOk(
                rollerMotor,
                5,
                () -> rollerMotor.configure(
                        config,
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters));
    }

    // Logging per accessor
    @Logged(name = "Velocity")
    public double getVelocity() {
        if (RobotBase.isSimulation()) {
            return this.motorSim.getAngularVelocityRPM();
        }
        return this.rollerEncoder.getVelocity();
    }

    @Logged(name = "Voltage")
    public double getVoltage() {
        if (RobotBase.isSimulation()) {
            return this.motorSim.getInputVoltage();
        }
        return this.rollerMotor.getBusVoltage() * this.rollerMotor.getAppliedOutput();
    }

    @Logged(name = "CurrentAmps")
    public double getOutputCurrent() {
        if (RobotBase.isSimulation()) {
            return this.motorSim.getCurrentDrawAmps();
        }
        return this.rollerMotor.getOutputCurrent();
    }

    /**
     * Run motor at specified voltage
     * 
     * @param volts - Applied Voltage
     */
    public void runVolts(double volts) {
        rollerMotor.setVoltage(volts);

        if (RobotBase.isSimulation()) {
            motorSim.setInputVoltage(volts);
        }
    }

    /**
     * Runs a specified voltage at `volts` until the Command ends
     * 
     * @param volts - Applied Voltage
     */
    public Command runVoltsCommand(double volts) {
        return Commands.runEnd(() -> this.runVolts(volts), () -> this.runVolts(0));
    }
}