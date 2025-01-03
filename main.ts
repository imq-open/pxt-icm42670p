
icm42670p.setSlaveAddress(0b1101000)

basic.forever(function () {
    if (0 == icm42670p.setupImu()) {
        serial.writeString("setup imu success\n")
    }
    basic.pause(1000)
})
