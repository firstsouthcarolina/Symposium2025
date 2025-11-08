package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

public class TurretConstants {
  public static final int kCanId = 6;

  // 25:1 MAX Cartridge combo with a 216 tooth gear over a 24 tooth turret
  public static final double kReduction = 25.0 * (216.0 / 24.0);

  public static final boolean kInverted = true;
  public static final double kSupplyCurrentLimitAmps = 30;

  public static final boolean kIsBrakeMode = true;

  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html#combined-feedforward-and-feedback-control
  public static final Slot0Configs kGains =
      new Slot0Configs()
          // feedforward
          .withKS(0.05)
          .withKV(0.05)
          .withKA(0.0)
          // feedback
          .withKP(1.0)
          .withKI(0.0)
          .withKD(0.0);

  public static final MotionMagicConfigs kMotionMagicConfig =
      new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(5.0 * kReduction)
          .withMotionMagicAcceleration(4.5 * kReduction)
          .withMotionMagicJerk(20.0 * kReduction);

  public static final boolean kEnableFoc = true;
}