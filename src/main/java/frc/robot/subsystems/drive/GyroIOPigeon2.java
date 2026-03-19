// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.Constants.SwerveConstants;
import java.util.Arrays;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(SwerveConstants.DrivetrainConstants.Pigeon2Id, SwerveConstants.kCANBus);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    if (SwerveConstants.DrivetrainConstants.Pigeon2Configs != null) {
      pigeon.getConfigurator().apply(SwerveConstants.DrivetrainConstants.Pigeon2Configs);
    } else {
      pigeon.getConfigurator().apply(new Pigeon2Configuration());
    }

    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps = drainTimestampQueue(yawTimestampQueue);
    inputs.odometryYawPositions = drainYawPositionQueue(yawPositionQueue);
  }

  private static double[] drainTimestampQueue(Queue<Double> queue) {
    int size = queue.size();
    double[] values = new double[size];
    for (int i = 0; i < size; i++) {
      Double value = queue.poll();
      if (value == null) {
        return Arrays.copyOf(values, i);
      }
      values[i] = value;
    }
    return values;
  }

  private static Rotation2d[] drainYawPositionQueue(Queue<Double> queue) {
    int size = queue.size();
    Rotation2d[] values = new Rotation2d[size];
    for (int i = 0; i < size; i++) {
      Double value = queue.poll();
      if (value == null) {
        return Arrays.copyOf(values, i);
      }
      values[i] = Rotation2d.fromDegrees(value);
    }
    return values;
  }
}
