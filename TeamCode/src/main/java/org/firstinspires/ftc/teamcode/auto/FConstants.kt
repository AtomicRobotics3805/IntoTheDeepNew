package org.firstinspires.ftc.teamcode.auto

import com.pedropathing.follower.FollowerConstants
import com.pedropathing.localization.Localizers
import com.pedropathing.localization.constants.TwoWheelConstants
import com.qualcomm.robotcore.hardware.DcMotorSimple


class FConstants {
    init {
        FollowerConstants.localizers = Localizers.TWO_WHEEL

        FollowerConstants.leftFrontMotorName = "LF"
        FollowerConstants.leftRearMotorName = "LB"
        FollowerConstants.rightFrontMotorName = "RF"
        FollowerConstants.rightRearMotorName = "RB"

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE // TODO: May need fixing
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE // TODO: May need fixing
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD // TODO: May need fixing
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD // TODO: May need fixing

        FollowerConstants.mass = 12.7913

        FollowerConstants.xMovement = 69.4109
        FollowerConstants.yMovement = 44.58105

        FollowerConstants.forwardZeroPowerAcceleration = -29.2229
        FollowerConstants.lateralZeroPowerAcceleration = -89.8578

        FollowerConstants.useSecondaryTranslationalPID = false
        FollowerConstants.useSecondaryHeadingPID = false
        FollowerConstants.useSecondaryDrivePID = true

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.2,0.0,0.035,0.0)
        FollowerConstants.headingPIDFCoefficients.setCoefficients(1.0,0.0,0.0,0.0)
        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.007,0.0,0.000025,0.6,0.0)
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.001,0.0,0.00001,0.6,0.0)


    }
}
