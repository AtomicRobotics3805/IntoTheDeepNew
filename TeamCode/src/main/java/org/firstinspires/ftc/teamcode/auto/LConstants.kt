package org.firstinspires.ftc.teamcode.auto

import com.pedropathing.localization.Encoder
import com.pedropathing.localization.constants.TwoWheelConstants
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot


object LConstants {
    init {
        TwoWheelConstants.forwardTicksToInches = 0.0 // TODO: TUNE
        TwoWheelConstants.strafeTicksToInches = 0.0 // TODO: TUNE

        TwoWheelConstants.forwardY = 4.4469
        TwoWheelConstants.strafeX = -5.3917

        TwoWheelConstants.IMU_Orientation = RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        )

        TwoWheelConstants.forwardEncoder_HardwareMapName = "RB"
        TwoWheelConstants.strafeEncoder_HardwareMapName = "LB"

        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD
    }
}
