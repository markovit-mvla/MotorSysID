/*
 * @author tohar
 * @brief  114 SysID tuning of motors
 */

#include <frc/PIDController.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Talon.h>
#include <frc/TimedRobot.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include "SwerveDrive.h"
#include "Constants.h"

class MotorSysID : public frc::TimedRobot 
{
    /* Used to initialize motor controllers & PIDController */
    void RobotInit() override 
    {
        /* Initialize motor controllers */
        m_leftMotors.emplace_back(new frc::Talon(0));
        m_leftMotors.emplace_back(new frc::Talon(1));
        m_rightMotors.emplace_back(new frc::Talon(2));
        m_rightMotors.emplace_back(new frc::Talon(3));

        /* Motor Groups */
        m_leftSpeedControllerGroup = 
            std::make_unique<frc::SpeedControllerGroup>(m_leftMotors);
        m_rightSpeedControllerGroup =
            std::make_unique<frc::SpeedControllerGroup>(m_rightMotors);

        /* Initialize PID Controllers */
        m_frontLeftController  = std::make_unique<frc::PIDController>
            (SwerveConstants::P, SwerveConstants::I, SwerveConstants::D);
        m_frontRightController = std::make_unique<frc::PIDController>
            (SwerveConstants::P, SwerveConstants::I, SwerveConstants::D);
        m_backLeftController   = std::make_unique<frc::PIDController>
            (SwerveConstants::P, SwerveConstants::I, SwerveConstants::D);
        m_backRightController  = std::make_unique<frc::PIDController>
            (SwerveConstants::P, SwerveConstants::I, SwerveConstants::D);

        /* kTolerance refers to the tolerance of the PID Controller. */
        /* This could also be considered the error range. */
        m_leftController->SetTolerance(kTolerance);
        m_rightController->SetTolerance(kTolerance);
    }

    void TeleopPeriodic override
    {
        /* Read encoder values from each motor */
        double frontLeftEncoder  = SwerveConstants::FLencoder;
        double frontRightEncoder = SwerveConstants::FRencoder;
        double backLeftEncoder   = SwerveConstants::BLencoder;
        double backRightEncoder  = SwerveConstants::BRencoder;

        /* SetPoint for each PID controller */
        m_frontLeftController.SetSetpoint(m_frontLeftSetpoint);
        m_frontRightController.SetSetpoint(m_frontRightSetpoint);
        m_backLeftController.SetSetpoint(m_backLeftSetpoint);
        m_backRightController.SetSetpoint(m_backRightSetpoint);

        /* Calculate error and adjust output for PID controllers */
        auto m_frontLeftOutput  = m_frontLeftController.Calculate(frontLeftEncoder);
        auto m_frontRightOutput = m_frontRightController.Calculate(frontRightEncoder);
        auto m_backLeftOutput   = m_backLeftController.Calculate(backleftEnconder);
        auto m_backRightOutput  = m_backRightController.Calculate(backRightEncoder);

        /* Set speed for each of the four motors */
        SwerveDrive::flModule_.setSpeedMotor(m_frontLeftOutput);
        SwerveDrive::frModule_.setSpeedMotor(m_frontRightOutput);
        SwerveDrive::blModule_.setSpeedMotor(m_backLeftOutput);
        SwerveDrive::brModule_.setSpeedMotor(m_backRightOutput);
    }

    private:
        static constexpr double kTolerance = 0.05; /* Error of 0.05 or less is ok. Dummy value, need to tune */

        /* Setpoints are the ideal values - These need to be changed based on this */
        double m_frontLeftSetpoint  = 0.0;
        double m_frontRightSetpoint = 0.0;
        double m_backLeftSetpoint   = 0.0;
        double m_backRightSetpoint  = 0.0;

        /* Motors */
        std::vector<frc::SpeedController*> m_leftMotors;
        std::vector<frc::SpeedController*> m_rightMotors;

        /* Motor Groups */
        std::unique_ptr<frc::SpeedControllerGroup> m_leftSpeedControllerGroup;
        std::unique_ptr<frc::SpeedControllerGroup> m_rightSpeedControllerGroup;

        /* PID Controllers */
        std::unique_ptr<frc::PIDController> m_frontLeftController;
        std::unique_ptr<frc::PIDController> m_frontRightController;
        std::unique_ptr<frc::PIDController> m_backLeftController;
        std::unique_ptr<frc::PIDController> m_backRightController;
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<MotorSysID>(); }
#endif