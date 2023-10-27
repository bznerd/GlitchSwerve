package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kOI;
import frc.robot.Constants.kSwerve;
import frc.robot.utilities.ChassisLimiter;
import frc.robot.utilities.MAXSwerve;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Swerve extends SubsystemBase {
  // NT Objects
  NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
  NetworkTable ntTable = ntInst.getTable("system/drivetrain");
  NetworkTable modulesTable = ntTable.getSubTable("modules");
  NetworkTable poseTable = ntTable.getSubTable("pose");

  // Hardware
  private final MAXSwerve frontLeft =
      new MAXSwerve(
          kSwerve.CANID.frontLeftDrive,
          kSwerve.CANID.frontLeftSteer,
          kSwerve.Offsets.frontLeft,
          modulesTable.getSubTable("FrontLeft"));

  private final MAXSwerve backLeft =
      new MAXSwerve(
          kSwerve.CANID.backLeftDrive,
          kSwerve.CANID.backLeftSteer,
          kSwerve.Offsets.backLeft,
          modulesTable.getSubTable("BackLeft"));

  private final MAXSwerve backRight =
      new MAXSwerve(
          kSwerve.CANID.backRightDrive,
          kSwerve.CANID.backRightSteer,
          kSwerve.Offsets.backRight,
          modulesTable.getSubTable("BackRight"));

  private final MAXSwerve frontRight =
      new MAXSwerve(
          kSwerve.CANID.frontRightDrive,
          kSwerve.CANID.frontRightSteer,
          kSwerve.Offsets.frontRight,
          modulesTable.getSubTable("FrontRight"));

  private final AHRS navX = new AHRS(kSwerve.navxPort);

  // Controls objects
  private final SwerveDrivePoseEstimator poseEstimator;
  private final ChassisLimiter limiter;
  private final PIDController xController =
      new PIDController(kSwerve.Auton.xP, 0, kSwerve.Auton.xD);
  private final PIDController yController =
      new PIDController(kSwerve.Auton.yP, 0, kSwerve.Auton.yD);
  private final PIDController zController =
      new PIDController(kSwerve.Auton.zP, 0, kSwerve.Auton.zD);
  private Rotation2d gyroOffset = new Rotation2d();
  private ChassisSpeeds chassisVelocity = new ChassisSpeeds();

  // Logging
  private final Field2d field2d = new Field2d();
  private final FieldObject2d autonRobot = field2d.getObject("Autonomous Pose");
  private final FieldObject2d autonPath = field2d.getObject("Autonomous Path");
  private final DoublePublisher rawGyroPub = ntTable.getDoubleTopic("Raw Gyro").publish();
  private final DoublePublisher offsetGyroPub = ntTable.getDoubleTopic("Offset Gyro").publish();
  private final DoubleArrayPublisher chassisVelPub =
      ntTable.getDoubleArrayTopic("Commanded Chassis Velocity").publish();
  private final DoubleArrayPublisher measuredVelPub =
      ntTable.getDoubleArrayTopic("Actual Chassis Velocity").publish();
  private final DoublePublisher autonXError = poseTable.getDoubleTopic("Path x Error").publish();
  private final DoublePublisher autonYError = poseTable.getDoubleTopic("Path y Error").publish();
  private final DoublePublisher autonZError = poseTable.getDoubleTopic("Path z Error").publish();
  private final DoubleArrayPublisher swerveStatesPub =
      ntTable.getDoubleArrayTopic("Swerve Module States").publish();

  // Simulation
  private final SimDeviceSim simNavX = new SimDeviceSim("navX-Sensor", 0);
  private final SimDouble simNavXYaw = simNavX.getDouble("Yaw");

  public Swerve() {
    limiter = new ChassisLimiter(kSwerve.maxTransAccel, kSwerve.maxAngAccel);
    zController.enableContinuousInput(0, 2 * Math.PI);
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kSwerve.kinematics,
            getGyroRaw(),
            new SwerveModulePosition[] {
              frontLeft.getPositon(),
              backLeft.getPositon(),
              backRight.getPositon(),
              frontRight.getPositon()
            },
            new Pose2d());

    // Register the Field2d object as a sendable
    SendableBuilderImpl builder = new SendableBuilderImpl();
    builder.setTable(poseTable);
    SendableRegistry.publish(field2d, builder);
    builder.startListeners();

    // Bind Path Follower command logging methods
    PPSwerveControllerCommand.setLoggingCallbacks(
        autonPath::setTrajectory,
        autonRobot::setPose,
        null,
        (translation, rotation) -> {
          autonXError.set(translation.getX());
          autonYError.set(translation.getY());
          autonZError.set(rotation.getRadians());
        });
  }

  // ---------- Commands ----------

  // Telop field oriented driver commands
  public Command teleopDriveCommand(
      DoubleSupplier xTranslation,
      DoubleSupplier yTranslation,
      DoubleSupplier zRotation,
      BooleanSupplier boost) {
    return this.run(
        () ->
            driveFO(
                joystickToChassis(
                    xTranslation.getAsDouble(),
                    yTranslation.getAsDouble(),
                    zRotation.getAsDouble(),
                    boost.getAsBoolean()),
                kSwerve.Teleop.closedLoop));
  }

  public Command semiAutoDriveCommand(
      DoubleSupplier xTranslation,
      DoubleSupplier yTranslation,
      Rotation2d heading,
      BooleanSupplier boost) {
    return this.run(
        () -> {
          var speeds =
              joystickToChassis(
                  xTranslation.getAsDouble(), yTranslation.getAsDouble(), 0, boost.getAsBoolean());
          speeds.omegaRadiansPerSecond =
              zController.calculate(getPose().getRotation().getRadians(), heading.getRadians());
          driveFO(speeds, false);
        });
  }
  // Put wheels into x configuration
  public Command xSwerveCommand() {
    return this.startEnd(this::xSwerve, () -> {});
  }

  // Zero the gyro
  public Command zeroGyroCommand() {
    return this.runOnce(this::zeroGyro);
  }

  // Reset the gyro with pose estimation (don't use without vision)
  public Command resetGyroCommand() {
    return this.runOnce(this::matchGyroToPose);
  }

  // ---------- Autonomous ----------

  // Follow a PathPlanner path
  public Command followPath(PathPlannerTrajectory path, boolean transformForAlliance) {
    return new PPSwerveControllerCommand(
        path,
        this::getPose,
        this.xController,
        this.yController,
        this.zController,
        (speeds) -> this.drive(speeds, true),
        transformForAlliance,
        this);
  }

  // Follow a PathPlanner path and trigger commands passed in the event map at event markers
  public Command followPathWithEvents(
      PathPlannerTrajectory path, HashMap<String, Command> eventMap, boolean transformForAlliance) {
    return new FollowPathWithEvents(
        followPath(path, transformForAlliance), path.getMarkers(), eventMap);
  }

  // Generate an on-the-fly path to reach a certain pose
  public Command driveToPoint(Pose2d goalPose) {
    return driveToPoint(goalPose, goalPose.getRotation());
  }

  // Generate an on-the-fly path to reach a certain pose with a given holonomic rotation
  public Command driveToPoint(Pose2d goalPose, Rotation2d holonomicRotation) {
    PathPlannerTrajectory path =
        PathPlanner.generatePath(
            new PathConstraints(kSwerve.Auton.maxVel, kSwerve.Auton.maxAccel),
            new PathPoint(
                getPose().getTranslation(),
                new Rotation2d(
                    chassisVelocity.vxMetersPerSecond, chassisVelocity.vyMetersPerSecond),
                getPose().getRotation(),
                Math.sqrt(
                    Math.pow(chassisVelocity.vxMetersPerSecond, 2)
                        + Math.pow(chassisVelocity.vyMetersPerSecond, 2))),
            new PathPoint(goalPose.getTranslation(), goalPose.getRotation(), holonomicRotation));

    return followPath(path, false);
  }

  // ---------- Public interface methods ----------

  // Drive field-oriented (optional flag for closed loop velocity control)
  public void driveFO(ChassisSpeeds fieldRelativeSpeeds, boolean closedLoopDrive) {
    var relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getGyro());
    drive(relativeSpeeds, closedLoopDrive);
  }

  // Drive chassis-oriented (optional flag for closed loop velocity control)
  public void drive(ChassisSpeeds speeds, boolean closedLoopDrive) {
    speeds = limiter.calculate(speeds);
    var targetStates = kSwerve.kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, kSwerve.kModule.maxWheelSpeed);

    chassisVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyro().unaryMinus());
    setStates(targetStates, closedLoopDrive);
  }

  // Set wheels to x configuration
  public void xSwerve() {
    frontLeft.setX();
    backLeft.setX();
    backRight.setX();
    frontRight.setX();
  }

  // Retrieve the pose estimation pose
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  // Set an initial pose for the pose estimator
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(getGyroRaw(), getPositions(), pose);
  }

  // Zero out the gyro (current heading becomes 0)
  public void zeroGyro() {
    gyroOffset = getGyroRaw();
  }

  // Reset gyro using pose estimator
  public void matchGyroToPose() {
    gyroOffset = getGyroRaw().minus(getPose().getRotation());
  }

  // ---------- Private hardware interface methods ----------

  // Get direct gyro reading as Rotation2d
  private Rotation2d getGyroRaw() {
    return navX.getRotation2d();
  }

  // Get software offset gyro angle
  private Rotation2d getGyro() {
    return getGyroRaw().minus(gyroOffset);
  }

  // Get gyro yaw rate (radians/s CCW +)
  private double getGyroYawRate() {
    return Units.degreesToRadians(-navX.getRate());
  }

  // Retrieve the positions (angle and distance traveled) for each swerve module
  private SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPositon(), backLeft.getPositon(), backRight.getPositon(), frontRight.getPositon()
    };
  }

  // Retrieve the state (velocity and heading) for each swerve module
  private SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(), backLeft.getState(), backRight.getState(), frontRight.getState()
    };
  }

  // Set desired states (angle and velocity) to each module with an optional flag to enable closed
  // loop control on velocity
  private void setStates(SwerveModuleState[] states, boolean closedLoopDrive) {
    frontLeft.setTargetState(states[0], closedLoopDrive);
    backLeft.setTargetState(states[1], closedLoopDrive);
    backRight.setTargetState(states[2], closedLoopDrive);
    frontRight.setTargetState(states[3], closedLoopDrive);
  }

  // ---------- Periodic ----------

  // Update pose estimator and log data
  @Override
  public void periodic() {
    simNavXYaw.set(
        simNavXYaw.get() + chassisVelocity.omegaRadiansPerSecond * -360 / (2 * Math.PI) * 0.02);
    poseEstimator.update(getGyroRaw(), getPositions());
    poseEstimator.addVisionMeasurement(getPose(), getGyroYawRate());
    log();
  }

  // ---------- Helpers ----------
  private ChassisSpeeds joystickToChassis(
      double xTranslation, double yTranslation, double zRotation, boolean boost) {

    // Apply deadzones
    if (Math.abs(xTranslation) <= kOI.translationDeadzone) xTranslation = 0;
    if (Math.abs(yTranslation) <= kOI.translationDeadzone) yTranslation = 0;
    if (Math.abs(zRotation) <= kOI.rotationDeadzone) zRotation = 0;

    // Square inputs for controlabitly
    xTranslation = Math.copySign(xTranslation * xTranslation, xTranslation);
    yTranslation = Math.copySign(yTranslation * yTranslation, yTranslation);
    zRotation = Math.copySign(zRotation * zRotation, zRotation);

    // Create a velocity vector (full speed is a unit vector)
    var translationVelocity = VecBuilder.fill(xTranslation, yTranslation);

    // Multiply velocity vector by max speed
    translationVelocity = translationVelocity.times(kSwerve.maxTransSpeed);

    // Contrain velocities to boost gain
    if (!boost) {
      translationVelocity = translationVelocity.times(kSwerve.Teleop.translationGain);
      zRotation *= kSwerve.maxAngSpeed * kSwerve.Teleop.rotationGain;
    }

    // Construct chassis speeds and return
    return new ChassisSpeeds(
        translationVelocity.get(0, 0), translationVelocity.get(1, 0), zRotation);
  }

  // Log data to network tables
  private void log() {
    frontLeft.updateNT();
    backLeft.updateNT();
    backRight.updateNT();
    frontRight.updateNT();

    rawGyroPub.set(getGyroRaw().getRadians());
    offsetGyroPub.set(getGyro().getRadians());
    // Send the chassis velocity as a double array (vel_x, vel_y, omega_z)
    chassisVelPub.set(
        new double[] {
          chassisVelocity.vxMetersPerSecond,
          chassisVelocity.vyMetersPerSecond,
          chassisVelocity.omegaRadiansPerSecond
        });
    var measuredVel =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            kSwerve.kinematics.toChassisSpeeds(getStates()), getGyro().unaryMinus());
    measuredVelPub.set(
        new double[] {
          measuredVel.vxMetersPerSecond, measuredVel.vyMetersPerSecond, getGyroYawRate()
        });

    swerveStatesPub.set(
        new double[] {
          frontLeft.getTargetState().angle.getRadians(),
              frontLeft.getTargetState().speedMetersPerSecond,
          backLeft.getTargetState().angle.getRadians(),
              backLeft.getTargetState().speedMetersPerSecond,
          backRight.getTargetState().angle.getRadians(),
              backRight.getTargetState().speedMetersPerSecond,
          frontRight.getTargetState().angle.getRadians(),
              frontRight.getTargetState().speedMetersPerSecond
        });

    field2d.setRobotPose(getPose());
  }
}
