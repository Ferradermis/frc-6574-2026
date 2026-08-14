# FRC Team 6574 — 2026 Robot Code

[![WPILib](https://img.shields.io/badge/WPILib-2026.2.1-blue)](https://docs.wpilib.org)
[![AdvantageKit](https://img.shields.io/badge/AdvantageKit-logging-orange)](https://github.com/Mechanical-Advantage/AdvantageKit)

Robot code for FRC Team 6574's 2026 competition robot. Built on the [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit) logging framework with a WPILib command-based architecture, CTRE Phoenix 6 motor controllers, and PathPlanner for autonomous routines.

---

## Table of Contents
- [Framework & Libraries](#framework--libraries)
- [Subsystems](#subsystems)
- [Commands](#commands)
- [Button Bindings](#button-bindings)
- [Autonomous](#autonomous)
- [Constants](#constants)
- [Building & Deploying](#building--deploying)

---

## Framework & Libraries

| Library | Purpose |
|---|---|
| **WPILib 2026.2.1** | Core robot framework (command-based) |
| **AdvantageKit** | Deterministic logging & replay |
| **CTRE Phoenix 6** | TalonFX motor controllers, CANcoder, Pigeon 2 |
| **PathPlanner** | Autonomous path following |
| **PhotonVision** | Vision processing (camera pose estimation) |
| **Limelight** | Vision sensor IO |
| **YAMS** | Smart motor controller abstraction layer |
| **maple-sim** | Physics simulation (swerve drive + game elements) |
| **Spotless** | Code formatting (Gradle) |

The project targets **Java 17** and runs on a **RoboRIO 2**.

---

## Subsystems

### Drive
`frc.robot.subsystems.drive.Drive`

Field-relative swerve drive using four TalonFX-based modules (CTRE `SwerveModuleConstants`). Pose estimation is fused with vision measurements from two cameras. Heading is provided by a Pigeon 2 IMU.

- **Hardware:** 4× Kraken X60 (drive) + 4× Kraken X60 (steer) + CANcoder per module + Pigeon 2
- **CAN Bus:** `Drive` (CANFD)
- **Free speed:** ~5.04 m/s at 12 V
- **Odometry thread:** Phoenix high-frequency odometry at 250 Hz (CANFD) / 100 Hz (CAN)
- **PathPlanner:** Fully integrated; Translation PID `0.5`, Rotation PID `2.5`

---

### IntakePivot
`frc.robot.subsystems.intake.IntakePivot`

Single-axis pivot that raises and lowers the intake assembly. Uses a closed-loop Arm controller (PID + ArmFeedforward) with soft limits.

- **Motor:** 1× Kraken X44 (TalonFX), CAN ID `17`, bus `Subsystem`
- **Gear ratio:** 21:1
- **Soft limits:** −10° (down) to 95° (up)
- **Starting position:** 90°
- **Key angles:** −8.5° (intake deployed) · 27° (stow) · 90° (home) · 134° (full retract)
- Tracks fuel count for simulation

---

### IntakeMainRoller
`frc.robot.subsystems.intake.IntakeMainRoller`

Dual-motor roller that draws game pieces into the robot. Velocity-controlled.

- **Motors:** 2× Kraken X44 (TalonFX), CAN IDs `15` (left) & `16` (right), bus `Subsystem`
- **Normal intake speed:** 2 500 RPM (teleop) / 4 000 RPM (auto)
- **Dump speed:** −3 000 RPM

---

### Transition
`frc.robot.subsystems.Transition`

Belt/roller that moves game pieces from the intake toward the shooter.

- **Motor:** 1× Kraken X44 (TalonFX), CAN ID `19`, bus `Subsystem`
- **Typical speeds:** −800 RPM (intake feed-in) · 600 RPM (shoot feed-out)

---

### Shooter
`frc.robot.subsystems.shooter.Shooter`

Dual flywheel shooter. Calculates an optimal launch velocity based on measured distance to the hub using pose estimation.

- **Motors:** 2× (left CAN ID `23`, right CAN ID `24`), bus `Subsystem`
- **Preset speeds:** 2 000 RPM (close) · 2 500 RPM (far)
- **Failsafe speed:** 3 000 RPM
- Knows hub positions for both alliances for distance-based velocity calculation

---

### ShooterPivot
`frc.robot.subsystems.shooter.ShooterPivot`

Adjusts the shooter hood angle to optimize trajectory.

- **Motor:** 1× Kraken X44 (TalonFX), CAN ID `22`, bus `Subsystem`
- **Soft limits:** 30° – 72°
- **Starting position:** 70°
- **Failsafe angle:** 60°
- Calculates an optimal angle based on distance to hub

---

### ShooterTransition
`frc.robot.subsystems.shooter.ShooterTransition`

Dual-motor transition belt feeding the shooter flywheels.

- **Motors:** 2× TalonFX, CAN IDs `20` (left) & `21` (right), bus `Subsystem`
- **Feed speed:** 1 500 RPM

---

### Vision
`frc.robot.subsystems.vision.Vision`

Dual-camera pose estimation pipeline that feeds corrections into the drive odometry.

- **Real robot:** 2× Limelight cameras
- **Simulation:** 2× PhotonVision simulated cameras
- Camera names and robot-to-camera transforms defined in `VisionConstants`

---

## Commands

All teleop commands live in `frc.robot.commands.TeleopCommands`.

### Intake Commands

| Command | Description |
|---|---|
| `Intake(speed)` | Lowers pivot to −5°, runs main roller at the given speed |
| `IntakeAuto(speed, transSpeed)` | Lowers pivot to −8.5°, runs roller for 3 s, then stows |
| `DelayedIntakeAuto(speed, transSpeed)` | Waits 2 s, then runs a 2.5 s intake cycle (used in auto) |
| `IntakeAutoStop` | Stops roller and transition, then stows the intake |
| `StowIntake` | Moves pivot to stow angle (27°), stops roller |
| `DumpFuel(speed, transSpeed)` | Lowers pivot to 10°, runs roller & transition in reverse to dump game pieces |

---

### Shooter Commands

| Command | Description |
|---|---|
| `StartShooter(drive)` | Spins shooter to optimal velocity (no transition feed) |
| `Shoot(drive)` | Full scoring sequence: spin up → aim → feed shooter transition → wiggle fuel into shooter |
| `ShootAuto(drive)` | `Shoot` with a 4 s timeout, then `StopShooter` |
| `ShootFailsafe(drive)` | Shoots at a fixed 3 000 RPM / 60° without pose-based targeting; uses `ShakeTheFuel` |
| `StopShooter(shooterSpeed, transSpeed, …)` | Stops transition belt, then shooter and transition motors |
| `Eject(…)` | Runs shooter/transition in reverse to clear a jam |

---

### Utility / Manipulation Commands

| Command | Description |
|---|---|
| `GoToHome` | Moves intake pivot to 134°, stops roller, moves shooter pivot to 70° |
| `WiggleFuel` | Rapidly oscillates intake pivot (10° → 50° → 10°) to settle game pieces before shooting |
| `WiggleFuelForDump` | Tighter oscillation (10° → 25° → 10°) for dump positioning |
| `ShakeTheFuel` | Oscillates pivot stow ↔ home (27° → 90° → 27°) to redistribute game pieces |

---

## Button Bindings

### Driver — Xbox Controller (port 0)

| Button / Axis | Action |
|---|---|
| Left stick | Field-relative translation (slew-rate limited) |
| Right stick X | Rotation (slew-rate limited) |
| **Left Bumper** | Stop & lock wheels in X-pattern |
| **Right Bumper** (hold) | `Intake` at 2 500 RPM; releases to 0 RPM |
| **A** (hold) | `Shoot` (optimal velocity + angle); releases → `StopShooter` |
| **Y** (hold) | `DumpFuel` at −3 000 RPM; releases to stop |
| **Left Trigger** (hold) | `Eject` (1 500 / −750 / −400 RPM); releases → `StopShooter` |

### Operator — Generic HID (port 1)

| Button | Action |
|---|---|
| **1** | `WiggleFuelForDump` |
| **2** | `GoToHome` |
| **3** (hold) | Drive locked to angle toward hub |
| **4** | `WiggleFuel` |
| **5** | `ShakeTheFuel` |
| **6** (hold) | `ShootFailsafe`; releases → `StopShooter` |

---

## Autonomous

Routines are built with **PathPlanner** and selected via a `LoggedDashboardChooser` on SmartDashboard / Shuffleboard.

### Named Commands (available in PathPlanner GUI)

| Name | Command |
|---|---|
| `Shoot` | `ShootAuto` (full shoot + stop) |
| `Shoot Without Stopping` | `Shoot` only |
| `Intake` | `IntakeAuto` at 4 000 RPM |
| `Delayed Intake` | `DelayedIntakeAuto` (2 s delay, then intake) |
| `StopIntake` | `IntakeAutoStop` |
| `Intake Down` | `Intake` at 4 000 RPM (0.75 s) |
| `Stow` | `StowIntake` (0.75 s) |
| `Small Stow` | Pivot to 105° (0.5 s) |
| `Shaky Shaky` | `ShakeTheFuel` (2 s) |
| `Wiggle Waggle` | `WiggleFuel` (2 s) |

### Pre-built Auto Paths
Paths are stored in `src/main/deploy/pathplanner/autos/`. Highlights include:
- Left/Right/Mid ramp shoot autos
- Depot collection + shoot sequences
- Trench + outpost combinations
- PathPlanner PID tuning routine

### SysId Routines (selectable in auto chooser)
- Drive wheel radius characterization
- Drive simple FF characterization
- Drive SysId quasistatic & dynamic (forward/reverse)

---

## Constants

All shared constants live in `frc.robot.Constants`.

### `MechanismConstants`

| Constant | Value | Description |
|---|---|---|
| `SHOOTER_SPEED_CLOSE` | 2 000 RPM | Shooter speed for close shots |
| `SHOOTER_SPEED_FAR` | 2 500 RPM | Shooter speed for far shots |
| `SHOOTER_TRANSITION_SPEED` | 1 500 RPM | Shooter transition feed speed |
| `TRANSITION_SPEED_SHOOT` | 600 RPM | Main transition speed during shoot |
| `TRANSITION_SPEED_INTAKE` | −800 RPM | Main transition speed during intake |
| `INTAKE_SPEED` | 1 000 RPM | Default intake speed |
| `INTAKE_SPEED_DUMP` | −1 000 RPM | Reverse intake for dump |
| `INTAKE_HOME` | 90° | Intake pivot home position |
| `INTAKE_STOW_ANGLE` | 27° | Intake pivot stow position |

### `CanIds` (all on `Subsystem` CAN bus)

| Constant | ID | Device |
|---|---|---|
| `INTAKE_MAIN_ROLLER_LEFT_ID` | 15 | Left intake roller |
| `INTAKE_MAIN_ROLLER_RIGHT_ID` | 16 | Right intake roller |
| `INTAKE_PIVOT_ID` | 17 | Intake pivot |
| `INTAKE_RAMP_PIVOT_ID` | 18 | Fuel ramp pivot |
| `TRANSITION_ID` | 19 | Transition belt |
| `SHOOTER_TRANSITION_LEFT_ID` | 20 | Left shooter transition |
| `SHOOTER_TRANSITION_RIGHT_ID` | 21 | Right shooter transition |
| `SHOOTER_HOOD_ID` | 22 | Shooter pivot (hood) |
| `SHOOTER_LEFT_ID` | 23 | Left shooter flywheel |
| `SHOOTER_RIGHT_ID` | 24 | Right shooter flywheel |

### `Dimensions`

| Constant | Value | Description |
|---|---|---|
| `FRAME_SIZE_X` | 27.5 in | Frame front-to-back |
| `FRAME_SIZE_Y` | 27.5 in | Frame left-to-right |
| `BUMPER_THICKNESS` | 3 in | Bumper protrusion |
| `BUMPER_HEIGHT` | 7 in | Bumper height from floor |

---

## Building & Deploying

### Build

```powershell
.\gradlew.bat build
```

### Deploy to Robot

```powershell
.\gradlew.bat deploy
```

### Simulate

```powershell
.\gradlew.bat simulateJava
```

> **Sim mode** uses maple-sim for physics simulation including a fuel-game-element simulation (`FuelSim`). Set `Constants.simMode = Mode.SIM` (default).

### Code Formatting (Spotless)

```powershell
.\gradlew.bat spotlessApply
```

---

## Project Structure

```
src/main/java/frc/robot/
├── commands/
│   └── TeleopCommands/     # All operator commands
├── generated/
│   └── TunerConstants.java # CTRE Tuner X swerve config (auto-generated)
├── subsystems/
│   ├── drive/              # Swerve drive + odometry
│   ├── intake/             # IntakePivot, IntakeMainRoller
│   ├── shooter/            # Shooter, ShooterPivot, ShooterTransition
│   ├── vision/             # Vision pipeline
│   └── Transition.java     # Main game-piece transition belt
├── util/                   # PhoenixUtil, FuelSim
├── Constants.java          # All robot-wide constants
├── RobotContainer.java     # Subsystems, button bindings, auto chooser
└── Robot.java              # AdvantageKit robot base
src/main/deploy/
└── pathplanner/            # Auto paths and settings
```
