/// <reference path="raw.ts" />


/**
 * ICM-42670-P driver
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

}
