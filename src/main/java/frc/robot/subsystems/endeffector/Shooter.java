// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

import com.ctre.phoenix6.hardware.CANcoder;

public class Shooter extends SubsystemBase {

  private static final double MAX_RPM = 3600.0;
  private static final double HOOD_COMMAND_EPSILON_DEGREES = 0.1;
  private static final double HOOD_MIN_ANGLE_DEGREES = -30.0;
  private static final double HOOD_MAX_ANGLE_DEGREES = 0.0;
  private static final int QUADRATIC_POINT_COUNT = 3;
  private static final LookupPoint[] LOOKUP_POINTS = {
    new LookupPoint(2.35, 1825.0, 00),
    new LookupPoint(3.11, 1875.0, 0),
    new LookupPoint(3.84, 2075.0, 0)
  };

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
  private final CANcoder hoodEncoder = new CANcoder(ShooterConstants.hoodEncoderID);
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
                                                    .startingAngle(hoodEncoder.getAbsolutePosition().getValueAsDouble())
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
     shooter.updateInputs(shooterInputs);
     indexer.updateInputs(indexerInputs);
    Logger.processInputs("PhaseShift", phaseshiftInputs);
    Logger.processInputs("Shooter", shooterInputs);
    Logger.processInputs("Indexer", indexerInputs);
    Logger.recordOutput("Shooter/DistanceToTargetMeters", getDistanceToTarget());
    Logger.recordOutput("Shooter/HoodAngle", hood.getAngle());
    Logger.recordOutput(
        "Shooter/ShotTuning/UseDashboardSetpoints", shouldUseDashboardShotTuning());
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

  /** Returns the quadratically interpolated RPM for the current distance to target, capped at MAX_RPM. */
  public double getLookupRpm() {
    double distance = getDistanceToTarget();
    double rpm = interpolateQuadratic(distance, LookupPoint::rpm);
    return Math.min(rpm, MAX_RPM);
  }

  /** Returns the quadratically interpolated hood angle for the current distance to target. */
  public double getLookupHoodAngle() {
    return interpolateQuadratic(getDistanceToTarget(), LookupPoint::hoodAngleDegrees);
  }

  public boolean shouldUseDashboardShotTuning() {
    return useDashboardShotTuning.get() > 0.5;
  }

  public double getDashboardShooterRpm() {
    return dashboardShooterRpm.get();
  }

  public double getDashboardHoodAngle() {
    double normalizedValue = MathUtil.clamp(dashboardHoodPercent.get() / 100.0, 0.0, 1.0);
    return HOOD_MAX_ANGLE_DEGREES
        - normalizedValue * (HOOD_MAX_ANGLE_DEGREES - HOOD_MIN_ANGLE_DEGREES);
  }

  public double getActiveTargetRpm() {
    return shouldUseDashboardShotTuning() ? getDashboardShooterRpm() : getLookupRpm();
  }

  public double getActiveTargetHoodAngle() {
    return shouldUseDashboardShotTuning() ? getDashboardHoodAngle() : getLookupHoodAngle();
  }

  private double interpolateQuadratic(double distance, LookupValueExtractor valueExtractor) {
    LookupPoint[] interpolationPoints = getClosestLookupPoints(distance);
    double interpolatedValue = 0.0;

    for (int i = 0; i < interpolationPoints.length; i++) {
      double xi = interpolationPoints[i].distanceMeters();
      double yi = valueExtractor.extract(interpolationPoints[i]);
      double basis = 1.0;

      for (int j = 0; j < interpolationPoints.length; j++) {
        if (i == j) {
          continue;
        }

        double xj = interpolationPoints[j].distanceMeters();
        basis *= (distance - xj) / (xi - xj);
      }

      interpolatedValue += yi * basis;
    }

    return interpolatedValue;
  }

  private LookupPoint[] getClosestLookupPoints(double distance) {
    return Arrays.stream(LOOKUP_POINTS)
        .sorted(Comparator.comparingDouble(point -> Math.abs(point.distanceMeters() - distance)))
        .limit(Math.min(QUADRATIC_POINT_COUNT, LOOKUP_POINTS.length))
        .sorted(Comparator.comparingDouble(LookupPoint::distanceMeters))
        .toArray(LookupPoint[]::new);
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

  private static record LookupPoint(double distanceMeters, double rpm, double hoodAngleDegrees) {}

  @FunctionalInterface
  private static interface LookupValueExtractor {
    double extract(LookupPoint point);
  }
}
