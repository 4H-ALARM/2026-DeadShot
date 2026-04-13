// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Constants.GenericConstants;
import frc.lib.Constants.ShooterConstants;
import frc.lib.catalyst.hardware.MotorType;
import frc.lib.catalyst.mechanisms.RotationalMechanism;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.commands.RumbleController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.targeting.ShootTargetIO;
import java.util.Arrays;
import java.util.Comparator;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class Shooter extends SubsystemBase {

  private static final double MAX_RPM = 3600.0;
  private static final double HOOD_COMMAND_EPSILON_DEGREES = 0.1;
  private static final double HOOD_MIN_ANGLE_DEGREES = -30.0;
  private static final double HOOD_MAX_ANGLE_DEGREES = 0.0;
  private static final LookupPoint[] LOOKUP_POINTS = {
    new LookupPoint(3.55, 2075*.95, 0.0),
    new LookupPoint(3.07, 1875*.95, 0.0),
    new LookupPoint(2.75, 1825*.95, 0.0),
    new LookupPoint(4.26, 2150*.95, 25.0),
    new LookupPoint(3.46, 2000*.95, 0.0),
    new LookupPoint(3.67, 2100*.95, 0.0),
    new LookupPoint(3.84, 2000*.95, 20.0),
    new LookupPoint(3.66, 1950*.95, 15.0),
    new LookupPoint(2.6, 1775*.95, 0.0),
    new LookupPoint(5.52, 2300*.95, 30.0),
    new LookupPoint(5.07, 2150*.95, 27.5)
  };
  private static final LookupPoint[] SORTED_LOOKUP_POINTS =
      Arrays.stream(LOOKUP_POINTS)
          .sorted(Comparator.comparingDouble(LookupPoint::distanceMeters))
          .toArray(LookupPoint[]::new);
  private static final double[] MONOTONIC_HOOD_PERCENTS = buildMonotonicHoodPercents();

  private ShooterIO shooter;
  private ShooterIOInputsAutoLogged shooterInputs;
  private Drive drive;
  private IndexerIO indexer;
  private IndexerIOInputsAutoLogged indexerInputs;
  private PhaseshiftIO phaseshift;
  private PhaseshiftIOInputsAutoLogged phaseshiftInputs;
  private ShootTargetIO shootTarget;
  private double lastPhaseTime = 0;
  private Command rumble10Seconds;
  private Command rumble3Seconds;
  private Command rumbleEndShift;
  private double lastCommandedHoodAngleDegrees = Double.NaN;
  private final CANcoder hoodEncoder = new CANcoder(ShooterConstants.hoodEncoderID, ShooterConstants.shooterCanbus);
  private final LoggedTunableNumber useDashboardShotTuning =
      new LoggedTunableNumber("Shooter/ShotTuning/UseDashboardSetpoints", 0.0);
  private final LoggedTunableNumber dashboardShooterRpm =
      new LoggedTunableNumber("Shooter/ShotTuning/ShooterRPM", 1825.0);
  private final LoggedTunableNumber dashboardHoodPercent =
      new LoggedTunableNumber("Shooter/ShotTuning/HoodPercent", 50.0);
  private RotationalMechanism hood;

  /** FIX DO NOT WANT TO IMPORT A WHOLE DRIVE */
  public Shooter(
      ShooterIO shooter,
      Drive drive,
      IndexerIO indexer,
      PhaseshiftIO phaseshift,
      ShootTargetIO shootTarget,
      CommandXboxController controller
      ) {
    this.shooter = shooter;
    this.drive = drive;
    this.indexer = indexer;
    this.phaseshift = phaseshift;
    this.shootTarget = shootTarget;
    this.phaseshiftInputs = new PhaseshiftIOInputsAutoLogged();
    this.shooterInputs = new ShooterIOInputsAutoLogged();
    this.indexerInputs = new IndexerIOInputsAutoLogged();
    this.rumble3Seconds = new RumbleController(controller, 3, 0.1);
    this.rumble10Seconds = new RumbleController(controller, 0.5, 0.1);
    this.rumbleEndShift = new RumbleController(controller, 1, 1);
    MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs().withMagnetOffset(-0.071533203125).withSensorDirection(SensorDirectionValue.Clockwise_Positive);
    hoodEncoder.getConfigurator().apply(magnetSensorConfigs);
    this.hood = new RotationalMechanism(RotationalMechanism.Config.builder()
                                                    .name("hood")
                                                    .canBus("endEffector")
                                                    .currentLimit(30)
                                                    .motor(ShooterConstants.hoodMotorID)
                                                    .follower(ShooterConstants.hoodMotorFollowerID, true)
                                                    .motorType(MotorType.KRAKEN_X60_FOC)
                                                    .gearRatio(213.3333)
                                                    .pid(900,0,8)
                                                    .feedforward(0, 20.85)
                                                    .range(HOOD_MIN_ANGLE_DEGREES, HOOD_MAX_ANGLE_DEGREES)
                                                    .statorCurrentLimit(30)
                                                    .startingAngle(hoodEncoder.getAbsolutePosition().getValue().in(Units.Degrees))
                                                    .motionMagic(999, 9999, 0).build());



    // Distance (meters) -> RPM calibration points for quadratic interpolation
    // TODO: tune these values with real testing
    // shootTarget.setTarget(GenericConstants.HUB_POSE3D, true);
  }

  @Override
  public void periodic() {
     phaseshift.updateInputs(phaseshiftInputs);

     if(phaseshiftInputs.phaseTimeRemaining <= 10 && lastPhaseTime > 10 && phaseshiftInputs.myHubActive == false){
      CommandScheduler.getInstance().schedule(rumble10Seconds);

     }
     if(phaseshiftInputs.phaseTimeRemaining <= 3 && lastPhaseTime > 3 && phaseshiftInputs.myHubActive == false){
        CommandScheduler.getInstance().schedule(rumble3Seconds);
     }
      if(phaseshiftInputs.phaseTimeRemaining <= 3 && lastPhaseTime > 3 && phaseshiftInputs.myHubActive == true){
        CommandScheduler.getInstance().schedule(rumbleEndShift);
     }
     lastPhaseTime = phaseshiftInputs.phaseTimeRemaining;
     shooter.updateTuningValues();
     shooter.updateInputs(shooterInputs);
     indexer.updateInputs(indexerInputs);
    Logger.processInputs("PhaseShift", phaseshiftInputs);
    Logger.processInputs("Shooter", shooterInputs);
    Logger.processInputs("Indexer", indexerInputs);
    Logger.recordOutput("Shooter/DistanceToTargetMeters", getDistanceToTarget());
    Logger.recordOutput("Shooter/HoodAngle", hood.getAngle());
    Logger.recordOutput("Shooter/ShotTuning/UseDashboardSetpoints", shouldUseDashboardShotTuning());
    Logger.recordOutput("Shooter/ShotTuning/HoodPercent", dashboardHoodPercent.get());
    Logger.recordOutput("Shooter/ShotTuning/TargetRPM", getActiveTargetRpm());
    Logger.recordOutput("Shooter/ShotTuning/TargetHoodAngle", getActiveTargetHoodAngle());
  }

  /** Returns the distance in meters from the robot to the current shoot target. */
  public double getDistanceToTarget() {
    Pose2d robotPose = drive.getPose();
    Translation2d targetXY = new Translation2d(shootTarget.getTarget().getX(), shootTarget.getTarget().getY());

    Logger.recordOutput("Shooter/targetpost", shootTarget.getTarget());
    return robotPose.getTranslation().getDistance(targetXY);
  }

  /** Returns the linearly interpolated RPM for the current distance to target, capped at MAX_RPM. */
  public double getLookupRpm() {
    double rpm = interpolateLookupValue(getDistanceToTarget(), LookupPoint::rpm);
    return Math.min(rpm, MAX_RPM);
  }

  /** Returns the interpolated hood angle in degrees for the current distance to target. */
  public double getLookupHoodAngle() {
    return hoodPercentToAngleDegrees(MathUtil.clamp(getLookupHoodPercent() / 100.0, 0.0, 1.0));
  }

  public boolean shouldUseDashboardShotTuning() {
    return useDashboardShotTuning.get() > 0.5;
  }

  public double getDashboardShooterRpm() {
    return dashboardShooterRpm.get();
  }

  public double getDashboardHoodAngle() {
    double normalizedValue = MathUtil.clamp(dashboardHoodPercent.get() / 100.0, 0.0, 1.0);
    return hoodPercentToAngleDegrees(normalizedValue);
  }

  public double getLookupHoodPercent() {
    return interpolateLookupValue(getDistanceToTarget(), MONOTONIC_HOOD_PERCENTS);
  }

  private double hoodPercentToAngleDegrees(double normalizedPercent) {
    return HOOD_MAX_ANGLE_DEGREES
        - normalizedPercent * (HOOD_MAX_ANGLE_DEGREES - HOOD_MIN_ANGLE_DEGREES);
  }

  public double getActiveTargetRpm() {
    return shouldUseDashboardShotTuning() ? getDashboardShooterRpm() : getLookupRpm();
  }

  public boolean isShooterAtTargetVelocity() {
    double targetRpm = getActiveTargetRpm();
    return Math.abs(getShooterVelocity() - targetRpm) <= ShooterConstants.shooterRevTolerance;
  }

  public double getActiveTargetHoodAngle() {
    return shouldUseDashboardShotTuning() ? getDashboardHoodAngle() : getLookupHoodAngle();
  }

  private double interpolateLookupValue(double distance, LookupValueExtractor valueExtractor) {
    if (distance <= SORTED_LOOKUP_POINTS[0].distanceMeters()) {
      return valueExtractor.extract(SORTED_LOOKUP_POINTS[0]);
    }

    for (int i = 1; i < SORTED_LOOKUP_POINTS.length; i++) {
      LookupPoint lowerPoint = SORTED_LOOKUP_POINTS[i - 1];
      LookupPoint upperPoint = SORTED_LOOKUP_POINTS[i];
      if (distance <= upperPoint.distanceMeters()) {
        double distanceSpan = upperPoint.distanceMeters() - lowerPoint.distanceMeters();
        if (distanceSpan <= 0.0) {
          return valueExtractor.extract(upperPoint);
        }

        double blend = (distance - lowerPoint.distanceMeters()) / distanceSpan;
        return MathUtil.interpolate(
            valueExtractor.extract(lowerPoint), valueExtractor.extract(upperPoint), blend);
      }
    }

    return valueExtractor.extract(SORTED_LOOKUP_POINTS[SORTED_LOOKUP_POINTS.length - 1]);
  }

  private double interpolateLookupValue(double distance, double[] values) {
    if (distance <= SORTED_LOOKUP_POINTS[0].distanceMeters()) {
      return values[0];
    }

    for (int i = 1; i < SORTED_LOOKUP_POINTS.length; i++) {
      LookupPoint lowerPoint = SORTED_LOOKUP_POINTS[i - 1];
      LookupPoint upperPoint = SORTED_LOOKUP_POINTS[i];
      if (distance <= upperPoint.distanceMeters()) {
        double distanceSpan = upperPoint.distanceMeters() - lowerPoint.distanceMeters();
        if (distanceSpan <= 0.0) {
          return values[i];
        }

        double blend = (distance - lowerPoint.distanceMeters()) / distanceSpan;
        return MathUtil.interpolate(values[i - 1], values[i], blend);
      }
    }

    return values[values.length - 1];
  }

  private static double[] buildMonotonicHoodPercents() {
    double[] monotonicPercents = new double[SORTED_LOOKUP_POINTS.length];
    double runningPercent = 0.0;
    for (int i = 0; i < SORTED_LOOKUP_POINTS.length; i++) {
      // Once hood starts coming in, keep it from dropping back to zero at farther distances.
      runningPercent =
          Math.max(runningPercent, MathUtil.clamp(SORTED_LOOKUP_POINTS[i].hoodPercent(), 0.0, 100.0));
      monotonicPercents[i] = runningPercent;
    }
    return monotonicPercents;
  }

  /** Applies the lookup-table setpoints based on distance to the current target. */
  public void applyLookupSetpoints() {
    double rpm = getActiveTargetRpm();
    setHoodAngle(getActiveTargetHoodAngle());
    shooter.setShooterSpeed(rpm / 60.0); // convert RPM to RPS
  }

  /** Spins the shooter at the lookup-table RPM based on distance to the current target. */
  public void spinShooterFromLookup() {
    applyLookupSetpoints();
  }

  public void spinShooter(double speed) {
    shooter.setShooterSpeed(speed);
  }

  public void stopShooter() {
    shooter.stopShooter();
  }

  public void setHoodAngle(double hoodAngle) {
    if (Double.isNaN(lastCommandedHoodAngleDegrees)
        || Math.abs(lastCommandedHoodAngleDegrees - hoodAngle) > HOOD_COMMAND_EPSILON_DEGREES) {
      CommandScheduler.getInstance().schedule(hood.goTo(hoodAngle));
      lastCommandedHoodAngleDegrees = hoodAngle;
    }
  }

  public RotationalMechanism getHood() {
    return this.hood;
  }

  public void setIndexerSpeed(double indexerSpeedInRPS) {
    indexer.setIndexerSpeed(indexerSpeedInRPS);
  }

  public void stopIndexer() {
    indexer.stopIndexer();
  }

  public Drive getDrive() {
    return drive;
  }

  public ShootTargetIO getShootTarget() {
    return shootTarget;
  }
  public double getShooterVelocity() {
    return shooter.getVelocity();
  }

  public void setTarget(Translation3d target) {
    shootTarget.setTarget(target, true);
  }

  public void resetTarget() {
    shootTarget.resetTarget();
  }

  private static record LookupPoint(double distanceMeters, double rpm, double hoodPercent) {}

  @FunctionalInterface
  private static interface LookupValueExtractor {
    double extract(LookupPoint point);
  }
}
