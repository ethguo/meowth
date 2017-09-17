import logging
import math
from time import time

import krpc
import serial

stagingDelay = 1

def scale(x, inLow, inHigh, outLow, outHigh):
	return (x - inLow) * (outHigh - outLow) / (inHigh - inLow) + outLow

def clamp(x, low, high):
	return max(low, min(high, x))

def scale_clamp(x, inLow, inHigh, outLow, outHigh):
	return clamp(scale(x, inLow, inHigh, outLow, outHigh), outLow, outHigh)

class SerialClient:
	def __init__(self):
		logging.info("Serial: Connecting")
		self.serial = serial.Serial("COM5", 9600, timeout=10)
		logging.info("Serial: Connected")

	def getData(self):
		raw_line = self.serial.readline()
		logging.debug("Serial: Read line: " + repr(raw_line))
		line = raw_line.decode("utf-8")
		line = line.split()
		line = [int(item) if item.find(".") == -1 else float(item) for item in line]
		if len(line) == 6:
			return line
		else:
			logging.warning("Serial: Corrupted line: " + repr(raw_line))


class KRPCClient:
	def __init__(self):
		logging.info("KRPC: Connecting")
		self.client = krpc.connect(name="Meowth")
		self.vessel = self.client.space_center.active_vessel
		self.vessel.auto_pilot.engage()
		logging.info("KRPC: Connected")

		self.lastStage = time()

	def stage(self):
		if time() - self.lastStage > stagingDelay:
			self.vessel.control.activate_next_stage()
			self.lastStage = time()

	def parachute(self):
		self.vessel.control.parachutes = True

	def setSteering(self, pitch, yaw):
		# pitch = clamp(pitch, -90, 90)
		# yaw = clamp(yaw, -90, 90)
		pitch2 = math.sqrt(pitch**2 + yaw**2)
		heading = (math.degrees(math.atan2(yaw, pitch))) % 360
		logging.info("(%8.4f, %8.4f) -> (%8.4f, %8.4f)" %(pitch, yaw, pitch2, heading))
		self.vessel.auto_pilot.target_pitch_and_heading(pitch2, heading)

	def setThrottle(self, throttle):
		self.vessel.control.throttle = scale_clamp(throttle, 100, 1023-100, 0, 1)

	def update(self, data):
		assert len(data) == 6
		stage, parachute, throttle, _, pitch, yaw,  = data

		self.setSteering(-pitch, -yaw)
		self.setThrottle(throttle)
		
		if stage:
			self.stage()

		if parachute:
			self.parachute()

	def getFuel(self):
		print(self.vessel.resources_in_decouple_stage(3).names)



def serialTest():
	s = SerialClient()
	while True:
		print(s.getData())

def main():
	c = KRPCClient()
	s = SerialClient()
	while True:
		data = s.getData()
		if data:
			c.update(data)


if __name__ == "__main__":
	logging.basicConfig(level=logging.INFO)
	# serialTest()
	main()
