package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

public class DrivetrainSubsystem extends SubsystemBase {
        private final SwerveModule m_frontLeftModule;
        private final SwerveModule m_frontRightModule;
        private final SwerveModule m_backLeftModule;
        private final SwerveModule m_backRightModule;
        private final SwerveDriveOdometry odometry;
        private final double[] drivePos = new double[4];
        private final AHRS m_navx = new AHRS(Port.kMXP);

        /**
         * The maximum voltage that will be delivered to the drive motors.
         * <p>
         * This can be reduced to cap the robot's maximum speed. Typically, this is
         * useful during initial testing of the robot.
         */
        public static final double MAX_VOLTAGE = 12;// 12
        /*
         Measure the drivetrain's maximum velocity or calculate the theoretical.
         The formula for calculating the theoretical maximum velocity is:
         <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
         pi
         By default this value is setup for a Mk3 standard module using Falcon500s to
         drive.
         An example of this constant for a Mk4 L2 module with NEOs to drive is:
         5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
         SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
         */
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 11000.0 / 60.0 *
                        SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
                        SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;

        // Here we calculate the theoretical maximum angular velocity. You can also
        // replace this with a measured amount.
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                        Math.hypot(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);
        // Creates our swerve kinematics using the robots track width and wheel base
        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Front right
                        new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back left
                        new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back right
                        new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

        private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        public DrivetrainSubsystem() {
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
                // The module has two NEOs on it. One for steering and one for driving.
                m_frontLeftModule = Mk3SwerveModuleHelper.createNeo(
                                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(0, 0),
                                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                                DrivetrainConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                DrivetrainConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
                                DrivetrainConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
                                DrivetrainConstants.FRONT_LEFT_MODULE_STEER_OFFSET);
                m_frontRightModule = Mk3SwerveModuleHelper.createNeo(
                                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(2, 0),
                                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                                DrivetrainConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                                DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                                DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);
                m_backLeftModule = Mk3SwerveModuleHelper.createNeo(
                                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(4, 0),
                                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                                DrivetrainConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                                DrivetrainConstants.BACK_LEFT_MODULE_STEER_MOTOR,
                                DrivetrainConstants.BACK_LEFT_MODULE_STEER_ENCODER,
                                DrivetrainConstants.BACK_LEFT_MODULE_STEER_OFFSET);
                m_backRightModule = Mk3SwerveModuleHelper.createNeo(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(6, 0),
                                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                                DrivetrainConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                DrivetrainConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
                                DrivetrainConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
                                DrivetrainConstants.BACK_RIGHT_MODULE_STEER_OFFSET);
                zeroGyroscope();

                odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation(),
                        new SwerveModulePosition[]{
                        });

        }

        public void zeroGyroscope() {
                m_navx.zeroYaw();
        }

        public Rotation2d getGyroscopeRotation() {
                return m_navx.getRotation2d();
        }

        public void ResetPos(Pose2d pos) {
                odometry.resetPosition(getGyroscopeRotation(),new SwerveModulePosition[]{
                        new SwerveModulePosition(drivePos[0], new Rotation2d(m_frontLeftModule.getSteerAngle())),
                        new SwerveModulePosition(drivePos[1], new Rotation2d(m_frontRightModule.getSteerAngle())),
                        new SwerveModulePosition(drivePos[2], new Rotation2d(m_backLeftModule.getSteerAngle())),
                        new SwerveModulePosition(drivePos[3], new Rotation2d(m_backRightModule.getSteerAngle()))
                } , pos);
        }

        public Pose2d getPos() {
                return odometry.getPoseMeters();
        }

        public void drive(ChassisSpeeds chassisSpeeds) {
                m_chassisSpeeds = chassisSpeeds;
        }

        private void setSpeeds(ChassisSpeeds chassisSpeeds) {
                SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

                setStates(states);
        }
        @Override
        public void periodic() {

                setSpeeds(m_chassisSpeeds);

                double updateSpeed = TimedRobot.kDefaultPeriod;

                drivePos[0] = drivePos[0] + (m_frontLeftModule.getDriveVelocity() * updateSpeed);
                drivePos[1] = drivePos[1] + (m_frontLeftModule.getDriveVelocity() * updateSpeed);
                drivePos[2] = drivePos[2] + (m_frontLeftModule.getDriveVelocity() * updateSpeed);
                drivePos[3] = drivePos[3] + (m_frontLeftModule.getDriveVelocity() * updateSpeed);

                updatePos();
        }

        private void updatePos() {
                odometry.update(getGyroscopeRotation(), new SwerveModulePosition[]{
                        new SwerveModulePosition(drivePos[0], new Rotation2d(m_frontLeftModule.getSteerAngle())),
                        new SwerveModulePosition(drivePos[1], new Rotation2d(m_frontRightModule.getSteerAngle())),
                        new SwerveModulePosition(drivePos[2], new Rotation2d(m_backLeftModule.getSteerAngle())),
                        new SwerveModulePosition(drivePos[3], new Rotation2d(m_backRightModule.getSteerAngle()))

                });
        }

        private void setStates(SwerveModuleState[] state) {
                m_frontLeftModule.set(
                        (state[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE) * AutoConstants.MAX_SPEED,
                        state[0].angle.getRadians());
                m_frontRightModule.set(
                        (state[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE) * AutoConstants.MAX_SPEED,
                        state[1].angle.getRadians());
                m_backLeftModule.set(
                        (state[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE) * AutoConstants.MAX_SPEED,
                        state[2].angle.getRadians());
                m_backRightModule.set(
                        (state[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE) * AutoConstants.MAX_SPEED,
                        state[3].angle.getRadians());
        }

        public Command getAuto() {
                List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("ForwardWithRot",
                                new PathConstraints(4, 3));

                HashMap<String, Command> eventMap = new HashMap<>();
                // eventMap.put("HalfwayPoint", new PrintCommand("Halfway through"));
                SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                                this::getPos, // Pose2d supplier
                                this::ResetPos, // Pose2d consumer, used to reset odometry at the beginning of auto
                                this.m_kinematics, // SwerveDriveKinematics
                                new PIDConstants(1.3, 0, 0), // PID constants to correct for translation error (used
                                                                 // to create the X and Y PID controllers)
                                new PIDConstants(.06, 0.0, 0.0), // PID constants to correct for rotation error (used to
                                                                 // create the rotation controller)
                                this::setStates, // Module states consumer used to output to the drive subsystem
                                eventMap,
                                this // The drive subsystem. Used to properly set the requirements of path following
                                     // commands
                );
                return autoBuilder.fullAuto(pathGroup);
        }
}
