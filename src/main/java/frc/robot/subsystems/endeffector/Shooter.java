// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Constants.ShooterConstants;
import frc.lib.catalyst.hardware.MotorType;
import frc.lib.catalyst.mechanisms.RotationalMechanism;
import frc.lib.firecontrol.ProjectileSimulator;
import frc.lib.firecontrol.ShotCalculator;
import frc.lib.firecontrol.ShotLUT;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.commands.RumbleController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.targeting.ShootTargetIO;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private static final double MAX_RPM = 3600.0;
  private static final double MIN_SOLUTION_CONFIDENCE = 50.0;
  private static final Rotation2d SHOOTER_FORWARD_OFFSET = new Rotation2d(Math.PI);

  // Projectile LUT model defaults. Tune to your mechanism as needed.
  private static final double LOOKUP_MIN_DISTANCE_METERS = 2.00;
  private static final double LOOKUP_MAX_DISTANCE_METERS = 6.00;
  private static final double LOOKUP_STEP_METERS = 0.1;
  private static final double BALL_MASS_KG = 0.2268;
  private static final double BALL_DIAMETER_M = 0.1778;
  private static final double DRAG_COEFFICIENT = 0.47;
  private static final double MAGNUS_COEFFICIENT = 0.4;
  private static final double AIR_DENSITY_KG_M3 = 1.225;
  private static final double EXIT_HEIGHT_METERS = 0.387;
  private static final double FLYWHEEL_DIAMETER_METERS = 0.1016;
  private static final double SLIP_FACTOR = 0.85;
  private static final double FIXED_LAUNCH_ANGLE_DEGREES = 67.0;
  private static final double SIMULATION_TIMESTEP_SECONDS = 0.001;
  private static final double MIN_SIM_RPM = 1500.0;
  private static final double MAX_SIM_RPM = MAX_RPM;
  private static final int RPM_SEARCH_ITERATIONS = 25;
  private static final double MAX_SIMULATION_TIME_SECONDS = 5.0;
  private static final double HOOD_SWEEP_MIN_ANGLE_DEGREES = 28;
  private static final double HOOD_SWEEP_MAX_ANGLE_DEGREES = 85.0;
  private static final double HOOD_SWEEP_STEP_DEGREES = 1.0;

  private static final double HOOD_COMMAND_EPSILON_DEGREES = 0.1;
  private static final double HOOD_MIN_ANGLE_DEGREES = -30.0;
  private static final double HOOD_MAX_ANGLE_DEGREES = 0.0;

  private final ShooterIO shooter;
  private final ShooterIOInputsAutoLogged shooterInputs;
  private final Drive drive;
  private final IndexerIO indexer;
  private final IndexerIOInputsAutoLogged indexerInputs;
  private final PhaseshiftIO phaseshift;
  private final PhaseshiftIOInputsAutoLogged phaseshiftInputs;
  private final ShootTargetIO shootTarget;
  private final Command rumble10Seconds;
  private final Command rumble3Seconds;
  private final Command rumbleEndShift;
  private final CANcoder hoodEncoder = new CANcoder(ShooterConstants.hoodEncoderID);
  private final ShotCalculator shotCalculator;
  private final LoggedTunableNumber useDashboardShotTuning =
      new LoggedTunableNumber("Shooter/ShotTuning/UseDashboardSetpoints", 0.0);
  private final LoggedTunableNumber dashboardShooterRpm =
      new LoggedTunableNumber("Shooter/ShotTuning/ShooterRPM", 1825.0);
  private final LoggedTunableNumber dashboardHoodPercent =
      new LoggedTunableNumber("Shooter/ShotTuning/HoodPercent", 50.0);
  private final LoggedTunableNumber shooterRevToleranceRpm =
      new LoggedTunableNumber(
          "Shooter/ShotTuning/ShooterRevToleranceRPM", ShooterConstants.shooterRevTolerance);

  private RotationalMechanism hood;
  private ShotCalculator.LaunchParameters latestLaunchParameters =
      ShotCalculator.LaunchParameters.INVALID;
  private double lastPhaseTime = 0;
  private double lastCommandedHoodAngleDegrees = Double.NaN;
  private int fireControlLutEntries = 0;

  public Shooter(
      ShooterIO shooter,
      Drive drive,
      IndexerIO indexer,
      PhaseshiftIO phaseshift,
      ShootTargetIO shootTarget,
      CommandXboxController controller) {
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

    MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs().withMagnetOffset(0.06884765625);

    hoodEncoder.getConfigurator().apply(magnetSensorConfigs);
    this.hood =
        new RotationalMechanism(
            RotationalMechanism.Config.builder()
                .name("hood")
                .canBus("endEffector")
                .currentLimit(30)
                .motor(ShooterConstants.hoodMotorID)
                .follower(ShooterConstants.hoodMotorFollowerID, true)
                .motorType(MotorType.KRAKEN_X60_FOC)
                .gearRatio(213.3333)
                .pid(900, 0, 8)
                .feedforward(0, 20.85)
                .range(HOOD_MIN_ANGLE_DEGREES, HOOD_MAX_ANGLE_DEGREES)
                .statorCurrentLimit(30)
                .startingAngle(0)
                .motionMagic(999, 9999, 0)
                .build());

    this.shotCalculator = createShotCalculator();
    generateFireControlLUT();
  }

  @Override
  public void periodic() {
    phaseshift.updateInputs(phaseshiftInputs);

    if (phaseshiftInputs.phaseTimeRemaining <= 10
        && lastPhaseTime > 10
        && phaseshiftInputs.myHubActive == false) {
      CommandScheduler.getInstance().schedule(rumble10Seconds);
    }
    if (phaseshiftInputs.phaseTimeRemaining <= 3
        && lastPhaseTime > 3
        && phaseshiftInputs.myHubActive == false) {
      CommandScheduler.getInstance().schedule(rumble3Seconds);
    }
    if (phaseshiftInputs.phaseTimeRemaining <= 3
        && lastPhaseTime > 3
        && phaseshiftInputs.myHubActive == true) {
      CommandScheduler.getInstance().schedule(rumbleEndShift);
    }

    lastPhaseTime = phaseshiftInputs.phaseTimeRemaining;
    shooter.updateInputs(shooterInputs);
    shooter.updateTuningValues();
    indexer.updateInputs(indexerInputs);
    latestLaunchParameters = calculateLaunchParameters();

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
    Logger.recordOutput("Shooter/ShotTuning/ShooterRevToleranceRPM", shooterRevToleranceRpm.get());
    Logger.recordOutput("Shooter/FireControl/HasValidLaunch", hasReliableLaunchSolution());
    Logger.recordOutput(
        "Shooter/FireControl/LaunchConfidence", latestLaunchParameters.confidence());
    Logger.recordOutput(
        "Shooter/FireControl/SolvedDistanceMeters", latestLaunchParameters.solvedDistanceM());
    Logger.recordOutput(
        "Shooter/FireControl/AimAngleRadians", getActiveTargetDriveAngle().getRadians());
    Logger.recordOutput("Shooter/FireControl/CalculatedRPM", latestLaunchParameters.rpm());
    Logger.recordOutput("Shooter/FireControl/ShotLUTSize", fireControlLutEntries);
  }

  public double getDistanceToTarget() {
    Pose2d robotPose = drive.getPose();
    Translation2d targetXY =
        new Translation2d(shootTarget.getTarget().getX(), shootTarget.getTarget().getY());
    Logger.recordOutput("Shooter/targetpost", shootTarget.getTarget());
    return robotPose.getTranslation().getDistance(targetXY);
  }

  public double getLookupRpm() {
    if (hasReliableLaunchSolution()) {
      return MathUtil.clamp(latestLaunchParameters.rpm(), 0.0, MAX_RPM);
    }
    if (fireControlLutEntries <= 0) {
      return 0.0;
    }
    return MathUtil.clamp(shotCalculator.getBaseRPM(getDistanceToTarget()), 0.0, MAX_RPM);
  }

  public double getLookupHoodAngle() {
    if (fireControlLutEntries <= 0) {
      return HOOD_MAX_ANGLE_DEGREES;
    }
    double lookupDistance =
        hasReliableLaunchSolution()
            ? latestLaunchParameters.solvedDistanceM()
            : getDistanceToTarget();
    double launchAngleDeg = shotCalculator.getHoodAngle(lookupDistance);
    Logger.recordOutput("Shooter/FireControl/LaunchAngleDeg", launchAngleDeg);
    double hoodEncoderAngleDeg = launchAngleToHoodEncoderAngle(launchAngleDeg);
    return MathUtil.clamp(hoodEncoderAngleDeg, HOOD_MIN_ANGLE_DEGREES, HOOD_MAX_ANGLE_DEGREES);
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
    double normalizedPercent =
        (HOOD_MAX_ANGLE_DEGREES - getLookupHoodAngle())
            / (HOOD_MAX_ANGLE_DEGREES - HOOD_MIN_ANGLE_DEGREES);
    return MathUtil.clamp(normalizedPercent, 0.0, 1.0) * 100.0;
  }

  private double hoodPercentToAngleDegrees(double normalizedPercent) {
    return HOOD_MAX_ANGLE_DEGREES
        - normalizedPercent * (HOOD_MAX_ANGLE_DEGREES - HOOD_MIN_ANGLE_DEGREES);
  }

  // Map launch-angle space (e.g. 37..67 deg) to hood encoder command space (-30..0 deg).
  private double launchAngleToHoodEncoderAngle(double launchAngleDeg) {
    double clampedLaunch =
        MathUtil.clamp(launchAngleDeg, HOOD_SWEEP_MIN_ANGLE_DEGREES, HOOD_SWEEP_MAX_ANGLE_DEGREES);
    double normalized =
        (clampedLaunch - HOOD_SWEEP_MIN_ANGLE_DEGREES)
            / (HOOD_SWEEP_MAX_ANGLE_DEGREES - HOOD_SWEEP_MIN_ANGLE_DEGREES);
    return MathUtil.interpolate(HOOD_MIN_ANGLE_DEGREES, HOOD_MAX_ANGLE_DEGREES, normalized);
  }

  public double getActiveTargetRpm() {
    return shouldUseDashboardShotTuning()
        ? getDashboardShooterRpm()
        : MathUtil.clamp(getLookupRpm(), 0.0, MAX_RPM);
  }

  public boolean isShooterAtTargetVelocity() {
    double targetRpm = getActiveTargetRpm();
    return Math.abs(getShooterVelocity() - targetRpm) <= shooterRevToleranceRpm.get();
  }

  public double getActiveTargetHoodAngle() {
    return shouldUseDashboardShotTuning() ? getDashboardHoodAngle() : getLookupHoodAngle();
  }

  public Rotation2d getActiveTargetDriveAngle() {
    if (hasReliableLaunchSolution()) {
      return latestLaunchParameters.driveAngle();
    }

    Translation2d robotXY = drive.getPose().getTranslation();
    Translation2d targetXY =
        new Translation2d(shootTarget.getTarget().getX(), shootTarget.getTarget().getY());
    Translation2d toTarget = targetXY.minus(robotXY);
    return new Rotation2d(toTarget.getX(), toTarget.getY()).rotateBy(SHOOTER_FORWARD_OFFSET);
  }

  private ShotCalculator createShotCalculator() {
    ShotCalculator.Config config = new ShotCalculator.Config();
    config.launcherOffsetX = -0.241; // rear-facing shooter is behind robot center
    config.launcherOffsetY = 0.0;
    config.minScoringDistance = LOOKUP_MIN_DISTANCE_METERS;
    config.maxScoringDistance = LOOKUP_MAX_DISTANCE_METERS;
    config.shooterAngleOffsetRad = Math.PI;
    return new ShotCalculator(config);
  }

  private void generateFireControlLUT() {
    Translation3d target = shootTarget.getTarget();
    ProjectileSimulator.SimParameters params =
        new ProjectileSimulator.SimParameters(
            BALL_MASS_KG,
            BALL_DIAMETER_M,
            DRAG_COEFFICIENT,
            MAGNUS_COEFFICIENT,
            AIR_DENSITY_KG_M3,
            EXIT_HEIGHT_METERS,
            FLYWHEEL_DIAMETER_METERS,
            target.getZ(),
            SLIP_FACTOR,
            FIXED_LAUNCH_ANGLE_DEGREES,
            SIMULATION_TIMESTEP_SECONDS,
            MIN_SIM_RPM,
            MAX_SIM_RPM,
            RPM_SEARCH_ITERATIONS,
            MAX_SIMULATION_TIME_SECONDS);

    // Backspin requested: Magnus sign -1.
    ProjectileSimulator simulator = new ProjectileSimulator(params, -1.0);

    // Adjustable-hood workflow from README.
    ShotLUT shotLut =
        simulator.generateVariableAngleShotLUT(
            HOOD_SWEEP_MIN_ANGLE_DEGREES,
            HOOD_SWEEP_MAX_ANGLE_DEGREES,
            HOOD_SWEEP_STEP_DEGREES);
    shotCalculator.loadShotLUT(shotLut);
    fireControlLutEntries = shotLut.size();

    // Also generate fixed-angle LUT stats on your requested 0.5m->6.0m range for visibility.
    ProjectileSimulator.GeneratedLUT generatedLUT =
        simulator.generateLUT(
            LOOKUP_MIN_DISTANCE_METERS, LOOKUP_MAX_DISTANCE_METERS, LOOKUP_STEP_METERS);
    Logger.recordOutput(
        "Shooter/FireControl/LUTReachablePoints", (double) generatedLUT.reachableCount());
    Logger.recordOutput(
        "Shooter/FireControl/LUTUnreachablePoints", (double) generatedLUT.unreachableCount());
    Logger.recordOutput("Shooter/FireControl/LUTMaxRangeMeters", generatedLUT.maxRangeM());
    Logger.recordOutput(
        "Shooter/FireControl/LUTGenerationTimeMs", (double) generatedLUT.generationTimeMs());
  }

  private ShotCalculator.LaunchParameters calculateLaunchParameters() {
    if (fireControlLutEntries <= 0) {
      return ShotCalculator.LaunchParameters.INVALID;
    }

    Pose2d pose = drive.getPose();
    ChassisSpeeds robotRelativeVelocity = drive.getChassisSpeeds();
    ChassisSpeeds fieldRelativeVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, drive.getRotation());
    Translation2d targetXY =
        new Translation2d(shootTarget.getTarget().getX(), shootTarget.getTarget().getY());

    ShotCalculator.ShotInputs shotInputs =
        new ShotCalculator.ShotInputs(
            pose,
            fieldRelativeVelocity,
            robotRelativeVelocity,
            targetXY,
            new Translation2d(1.0, 0.0),
            1.0,
            0.0,
            0.0);
    return shotCalculator.calculate(shotInputs);
  }

  private boolean hasReliableLaunchSolution() {
    return latestLaunchParameters.isValid()
        && latestLaunchParameters.confidence() >= MIN_SOLUTION_CONFIDENCE;
  }

  public void applyLookupSetpoints() {
    double rpm = getActiveTargetRpm();
    setHoodAngle(getActiveTargetHoodAngle());
    shooter.setShooterSpeed(rpm / 60.0); // RPM -> RPS
  }

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
}
