
//% 
namespace icm42670p {

    //%
    export enum Axis {
        X = 0,
        Y,
        Z,
    }

    /**
     *  Poll sensor data in an internal loop. Normally this function does not return 
     * unless some serious failure occurs.
     *  
     * @returns if something goes really wrong the loop breaks and this 
     *  function returns an error code
     */
    //% blockId=icm42670p_poll
    //% block="icm42670p poll loop" 
    //% shim=icm42670p::main
    export function poll(): number {
        return 0
    }

    /**
     * Set if data are read from FIFO (true) or registers (false)
     * 
     * @param enabled 
     */
    //% blockId=icm42670p_set_fifo_enabled
    //% block="icm42670p set fifo enabled $enabled"
    //% shim=icm42670p::set_fifo_enabled
    export function setFifoEnabled(enabled: boolean) {
        return
    }

    /**
     * Set if Highres is enabled. Only apply if FIFO is enabled.
     * 
     * @param enabled 
     */
    //% blockId=icm42670p_set_highres_enabled
    //% block="icm42670p set highres enabled $enabled"
    //% shim=icm42670p::set_highres_enabled
    export function setHighresEnabled(enabled: boolean) {
        return
    }


    /**
     * Set if power mode is low noise (true) or low power (false)
     * 
     * @param enabled 
     */
    //% blockId=icm42670p_set_ln_enabled
    //% block="icm42670p set low noise enabled $enabled"
    //% shim=icm42670p::set_ln_enabled
    export function setLowNoiseEnabled(enabled: boolean) {
        return
    }

    /**
     *  Get acceleration data as a Buffer.  
     * 
     * The returned Buffer object contains 3 data items, which are the acceleration on the X/Y/Z axes,
     * in that order:
     * 
     * - raw data: the data type is NumberFormat.Int32LE 
     * - converted: the data type is NumberFormat.Float32LE, in g (standard gravity)
     * 
     * @param raw get raw data or convert to g
     */
    //% blockId=icm42670p_get_accel_data
    //% block="icm42670p get accel data: raw? $raw"
    //% shim=icm42670p::get_accel_data
    export function getAccelData(raw: boolean): Buffer {
        return null
    }

    /**
     *  Get gyro data as a Buffer.
     * 
     * The returned Buffer object contains 3 data items, which are the angular speeds on the X/Y/Z axes,
     * in that order: 
     * 
     * - raw data: the data type is NumberFormat.Int32LE 
     * - converted: the data type is NumberFormat.Float32LE, in dps (degrees per second)
     * 
     * @param raw get raw data or convert to dps
     */
    //% blockId=icm42670p_get_gyro_data
    //% block="icm42670p get gyro data: raw? $raw"
    //% shim=icm42670p::get_gyro_data
    export function getGyroData(raw: boolean): Buffer {
        return null
    }


    /**
     *  Get temperature
     * 
     * @param raw get raw data or convert to degree celsius (°C)
     */
    //% blockId=icm42670p_get_temp_data
    //% block = "icm42670p get temperature: raw? $raw"
    //% shim=icm42670p::get_temp_data
    export function getTempData(raw: boolean): number {
        return 0
    }


}
