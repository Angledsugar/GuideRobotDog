from Phidget22.Phidget import *
from Phidget22.Devices.Accelerometer import *
from Phidget22.Devices.Gyroscope import *
from Phidget22.Devices.Magnetometer import *
from Phidget22.Devices.Spatial import *
import time

def onAccelerationChange(self, acceleration, timestamp):
	print("Acceleration: \t"+ str(acceleration[0])+ "  |  "+ str(acceleration[1])+ "  |  "+ str(acceleration[2]))
	print("Timestamp: " + str(timestamp))
	print("----------")

def onAngularRateUpdate(self, angularRate, timestamp):
	print("AngularRate: \t"+ str(angularRate[0])+ "  |  "+ str(angularRate[1])+ "  |  "+ str(angularRate[2]))
	print("Timestamp: " + str(timestamp))
	print("----------")

def onMagneticFieldChange(self, magneticField, timestamp):
	print("MagneticField: \t"+ str(magneticField[0])+ "  |  "+ str(magneticField[1])+ "  |  "+ str(magneticField[2]))
	print("Timestamp: " + str(timestamp))
	print("----------")

def onSpatialData(self, acceleration, angularRate, magneticField, timestamp):
	print("Acceleration: \t"+ str(acceleration[0])+ "  |  "+ str(acceleration[1])+ "  |  "+ str(acceleration[2]))
	print("AngularRate: \t"+ str(angularRate[0])+ "  |  "+ str(angularRate[1])+ "  |  "+ str(angularRate[2]))
	print("MagneticField: \t"+ str(magneticField[0])+ "  |  "+ str(magneticField[1])+ "  |  "+ str(magneticField[2]))
	print("Timestamp: " + str(timestamp))
	print("----------")

def main():
	accelerometer0 = Accelerometer()
	gyroscope0 = Gyroscope()
	magnetometer0 = Magnetometer()
	spatial0 = Spatial()

	accelerometer0.setOnAccelerationChangeHandler(onAccelerationChange)
	gyroscope0.setOnAngularRateUpdateHandler(onAngularRateUpdate)
	magnetometer0.setOnMagneticFieldChangeHandler(onMagneticFieldChange)
	spatial0.setOnSpatialDataHandler(onSpatialData)

	accelerometer0.openWaitForAttachment(5000)
	gyroscope0.openWaitForAttachment(5000)
	magnetometer0.openWaitForAttachment(5000)
	spatial0.openWaitForAttachment(5000)

	try:
		input("Press Enter to Stop\n")
	except (Exception, KeyboardInterrupt):
		pass

	accelerometer0.close()
	gyroscope0.close()
	magnetometer0.close()
	spatial0.close()

main()