/// <reference path="raw.ts" />


/**
 * ICM-42670-P driver
 */
//% weight=100 color=190 icon=""
//% advanced=true block="icm42670p"
namespace icm42670p {

    /**
     * Set 7-bit slave address
     */
    //% blockId=icm42670p_set_slave_address
    //% block="set slave address $addr"
    //% shim=icm42670p::set_slave_address
    export function setSlaveAddress(addr: number) {
        return
    }

}
