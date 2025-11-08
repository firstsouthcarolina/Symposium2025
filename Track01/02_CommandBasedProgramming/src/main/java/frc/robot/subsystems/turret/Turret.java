package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhoenixConstants;

public class Turret extends SubsystemBase {
  private static final double kNearGoalDegrees = 1;

  private final TalonFX talon = new TalonFX(TurretConstants.kCanId);

  private final PositionVoltage positionVoltage = new PositionVoltage(0)
      .withEnableFOC(TurretConstants.kEnableFoc);

  private final StatusSignal<Angle> position = talon.getPosition();

  // Not to be confused with the goal
  private final StatusSignal<Double> setpoint = talon.getClosedLoopReference();

  public Turret() {
    // Motor Configuration
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Slot0 = TurretConstants.kGains;

    cfg.MotionMagic = TurretConstants.kMotionMagicConfig;

    cfg.CurrentLimits
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(TurretConstants.kSupplyCurrentLimitAmps);

    cfg.MotorOutput
        .withNeutralMode(TurretConstants.kIsBrakeMode
            ? NeutralModeValue.Brake
            : NeutralModeValue.Coast)
        .withInverted(TurretConstants.kInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive);

    talon.getConfigurator().apply(cfg);

    // To optimize CANBus utilization
    BaseStatusSignal.setUpdateFrequencyForAll(
        PhoenixConstants.kStatusSignalUpdateFrequencyHz,
        position,
        setpoint);
    talon.optimizeBusUtilization(0.0, 1.0);

    // Subsystem Configuration
    resetAngle(TurretPosition.INITIAL.position);
    setGoal(TurretPosition.INITIAL.position);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        position,
        setpoint);
  }

  private Rotation2d getAngle() {
    return new Rotation2d(
        position.getValue().div(TurretConstants.kReduction));
  }

  private void resetAngle(Rotation2d angle) {
    talon.setPosition(angle.getRotations() * TurretConstants.kReduction);
  }

  private Rotation2d getSetpoint() {
    return new Rotation2d(setpoint.getValueAsDouble() / TurretConstants.kReduction);
  }

  private Rotation2d getGoal() {
    return new Rotation2d(
        positionVoltage.getPositionMeasure().div(TurretConstants.kReduction));
  }

  private void setGoal(Rotation2d goal) {
    talon.setControl(
        positionVoltage.withPosition(
            goal.getRotations() * TurretConstants.kReduction));
  }

  public Command setGoalCommand(Rotation2d goal) {
    return Commands.runOnce(() -> setGoal(goal), this);
  }

  public Command setGoalAndWaitCommand(Rotation2d goal) {
    return Commands.sequence(
        setGoalCommand(goal),
        Commands.waitUntil(this::isNearGoal));
  }

  private boolean isNearGoal() {
    return MathUtil.isNear(getGoal().getDegrees(), getAngle().getDegrees(), Turret.kNearGoalDegrees);
  }
}