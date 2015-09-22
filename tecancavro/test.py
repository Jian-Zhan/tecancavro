from transport import TecanAPISerial

print TecanAPISerial.findSerialPumps(tecan_addrs=[0], ser_baud=38400)

com_link = TecanAPISerial(tecan_addr = 0,
                          ser_port = "/dev/ttyS3",
                          ser_baud = 38400)

from XL3000 import XL3000
xl3000 = XL3000(com_link,
	            num_ports=8,
	            syringe_ul=1000,
                microstep=True,
                debug=True, debug_log_path='.')
xl3000.init(init_port=8)
print xl3000.getCurPort()

xl3000.changePort(2)
print xl3000.getCurPort()

xl3000.changePort(3)
print xl3000.getCurPort()

print xl3000.getPlungerPos(), xl3000.getEncoderPos()
xl3000.movePlungerAbs(3000)
print xl3000.getPlungerPos(), xl3000.getEncoderPos()

xl3000.aspirate(2, 500, execute = False)
xl3000.dispense(3, 200, execute = False)
xl3000.aspirate(2, 500, execute = False)
xl3000.dispense(3, 100, execute = False)
xl3000.dispenseAll(8, execute = False)
xl3000.executeChain(wait_ready = True)

xl3000.transfer(2, 8, 2400,  execute = False)
xl3000.executeChain(wait_ready = True)
print xl3000.getCurVolume(), xl3000.getCurPort()