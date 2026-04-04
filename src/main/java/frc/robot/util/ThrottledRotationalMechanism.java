package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.catalyst.mechanisms.RotationalMechanism;

/**
 * Reduces nonessential mechanism telemetry load without changing on-controller closed-loop
 * behavior. Motion Magic continues running on the motor controller between periodic updates.
 */
public class ThrottledRotationalMechanism extends RotationalMechanism {
  private final int enabledLoopDivisor;
  private final int disabledLoopDivisor;
  private int loopCounter = 0;

  public ThrottledRotationalMechanism(
      Config config, int enabledLoopDivisor, int disabledLoopDivisor) {
    super(config);
    this.enabledLoopDivisor = Math.max(1, enabledLoopDivisor);
    this.disabledLoopDivisor = Math.max(1, disabledLoopDivisor);
  }

  @Override
  public void periodic() {
    int loopDivisor = DriverStation.isDisabled() ? disabledLoopDivisor : enabledLoopDivisor;
    if ((loopCounter++ % loopDivisor) == 0) {
      super.periodic();
    }
  }
}
