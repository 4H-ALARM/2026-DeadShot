package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.endeffector.PhaseshiftIO.PhaseshiftIOInputs.AutoWinner;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;

public class PhaseshiftIO {
  // time before next phase is active where the hub will register scoring fuel.
  public static final double graceStartPeriod = 0;
  // time after current phase where the hub will register scoring fuel.
  public static final double graceEndPeriod = 3;

  @AutoLog
  public static class PhaseshiftIOInputs {
    public enum AutoWinner {
      UNDETERMINED,
      RED_ALLIANCE,
      BLUE_ALLIANCE
    };

    public AutoWinner autoWinner;
    public boolean myHubActive;
    public double phaseTimeRemaining;
  }

  public void updateInputs(PhaseshiftIOInputs inputs) {

    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      inputs.myHubActive = false;
      inputs.autoWinner = AutoWinner.UNDETERMINED;
      return;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      inputs.myHubActive = true;
      inputs.autoWinner = AutoWinner.UNDETERMINED;
      return;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      inputs.myHubActive = false;
      inputs.autoWinner = AutoWinner.UNDETERMINED;
      return;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in teleop.
    if (gameData.isEmpty()) {
      inputs.myHubActive = true;
      inputs.autoWinner = AutoWinner.UNDETERMINED;
      return;
    }

    switch (gameData.charAt(0)) {
      case 'R' -> inputs.autoWinner = AutoWinner.RED_ALLIANCE;
      case 'B' -> inputs.autoWinner = AutoWinner.BLUE_ALLIANCE;
      default -> {
        // If we have invalid game data, assume hub is active.
        inputs.myHubActive = true;
        inputs.autoWinner = AutoWinner.UNDETERMINED;
        return;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active =
        switch (alliance.get()) {
          case Red -> inputs.autoWinner == AutoWinner.BLUE_ALLIANCE;
          case Blue -> inputs.autoWinner == AutoWinner.RED_ALLIANCE;
        };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      inputs.myHubActive = true;
      inputs.phaseTimeRemaining = matchTime - 130;
    } else if (matchTime > 105) {
      // Shift 1
      inputs.myHubActive = shift1Active;
      inputs.phaseTimeRemaining = matchTime - 105;

    } else if (matchTime > 80) {
      // Shift 2
      inputs.myHubActive = !shift1Active;
      inputs.phaseTimeRemaining = matchTime - 80;
    } else if (matchTime > 55) {
      // Shift 3
      inputs.myHubActive = shift1Active;
      inputs.phaseTimeRemaining = matchTime - 55;
    } else if (matchTime > 30) {
      // Shift 4
      inputs.myHubActive = !shift1Active;
      inputs.phaseTimeRemaining = matchTime - 30;
    } else {
      // End game, hub always active.
      inputs.myHubActive = true;
      inputs.phaseTimeRemaining = matchTime;
    }
  }
}
