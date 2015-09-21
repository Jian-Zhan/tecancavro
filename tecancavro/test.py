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

xl3000.changePort(2, execute=True, wait_ready=False)
print xl3000.getCurPort()

xl3000.changePort(2, execute=True, wait_ready=True)
print xl3000.getCurPort()