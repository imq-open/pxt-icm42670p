

/**
 * ICM-42670-P/ICM-42607-P driver
 */
//% weight=100 color=190 icon=""
//% advanced=true block="icm42670p"
namespace icm42670p {

    /**
     * 7-bit slave address
     */
    export enum SlaveAddress {
        /**
         * Slave address when AD0 pin is low
         */
        AD0_0 = 0b1101000,
        /**
         * Slave address when AD0 pin is high
         */
        AD0_1,
    }

    /**
     * Set 7-bit slave address
     */
    //% blockId=icm42670p_set_slave_address
    //% block="icm42670p set slave address $addr"
    //% shim=icm42670p::set_slave_address
    export function setSlaveAddress(addr: SlaveAddress) {
        return
    }

    /**
     *  Set int1 pin
     */
    //% block="icm42670p set int1 pin $pin"
    //% shim=icm42670p::set_int1_pin
    export function setInt1Pin(pin: DigitalPin) {
        return
    }

    /**
     *  Poll sensor data in an internal loop. Return an error code on fatal error 
     */
    //% blockId=icm42670p_poll block="icm42670p poll"
    //% shim=icm42670p::poll
    export function poll(): number {
        return 0
    }

    /**
     *  Get Euler angles. The Euler angles yaw, pitch and roll are defined as
     * the angles of rotation around the Z, X and Y axis respectively.
     * 
     * Note that the yaw value is meaningless due to hardware and algorithm.
     */
    //% blockId=icm42670p_get_euler_angles block="icm42670p get Euler angles"
    //% shim=icm42670p::get_euler_angles
    export function getEulerAngles(): Buffer {
        return null
    }

    /**
     * Get pitch angle
     * @returns 
     */
    //% blockId=icm42670p_get_pitch_angle
    //% block="icm42670p get pitch angle"
    //% shim=icm42670p::get_pitch_angle
    export function getPitchAngle(): number {
        return 0
    }

    /**
     * Get roll angle
     * @returns 
     */
    //% blockId=icm42670p_get_roll_angle
    //% block="icm42670p get roll angle"
    //% shim=icm42670p::get_roll_angle
    export function getRollAngle(): number {
        return 0
    }

    /**
     *  Get temperature in degrees C
     */
    //% blockId=icm42670p_get_temp_deg_c
    //% block="icm42670p get temperature in degree C"
    //% shim=icm42670p::get_temp_deg_c
    export function getTempDegC(): number {
        return 0
    }

    /**
     *  Get accelerometer data in g
     */
    //% blockId=icm42670p_get_accel_g block="icm42670p get accelerometer data (in g)"
    //% shim=icm42670p::get_accel_g
    export function getAccelG(): Buffer {
        return null
    }

    /**
     * Get gyroscope data in dps (degrees per second)
     */
    //% blockId=icm42670p_get_gyro_dps
    //% block="icm42670p get gyroscope data (in dps)"
    //% shim=icm42670p::get_gyro_dps
    export function getGyroDps(): Buffer {
        return null
    }

} // namespace icm42670p 
