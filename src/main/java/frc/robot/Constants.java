// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Dimensions {
        public static final Distance BUMPER_THICKNESS = Inches.of(3); // frame to edge of bumper
        public static final Distance BUMPER_HEIGHT = Inches.of(7); // height from floor to top of bumper
        public static final Distance FRAME_SIZE_Y = Inches.of(27.5); // left to right (y-axis)
        public static final Distance FRAME_SIZE_X = Inches.of(27.5); // front to back (x-axis)
        public static final Distance FULL_WIDTH = FRAME_SIZE_Y.plus(BUMPER_THICKNESS.times(2));
        public static final Distance FULL_LENGTH = FRAME_SIZE_X.plus(BUMPER_THICKNESS.times(2));
    }

  public static final Pose3d HUB_CENTER_BLUE = new Pose3d(
    Units.inchesToMeters(182.11), 
    Units.inchesToMeters(158.84), 
    Units.inchesToMeters(72), 
    Rotation3d.kZero);

  public static final Pose3d HUB_CENTER_RED = new Pose3d(
    Units.inchesToMeters(469.11), 
    Units.inchesToMeters(158.84), 
    Units.inchesToMeters(72), 
    Rotation3d.kZero);
  

  public static class CanIds {
    public static final CANBus MECH_BUS = new CANBus("Subsystem");

    public static final int INTAKE_MAIN_ROLLERS_ID = 15;
    public static final int INTAKE_STATIC_ROLLER_ID = 16;
    public static final int INTAKE_PIVOT_ID = 17;
    public static final int INTAKE_RAMP_PIVOT_ID = 18;
    public static final int TRANSITION_ID = 19;
    public static final int SHOOTER_TRANSITION_LEFT_ID = 20;
    public static final int SHOOTER_TRANSITION_RIGHT_ID = 21;
    public static final int SHOOTER_HOOD_ID = 22;
    public static final int SHOOTER_LEFT_ID = 23;
    public static final int SHOOTER_RIGHT_ID = 24;
  }
}
