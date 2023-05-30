import spi
from enum import Enum
import spidev
import RPi.GPIO as GPIO
import time

class StatusCode(Enum):
		STATUS_OK = 0
		STATUS_ERROR = 1
		STATUS_COLLISION = 2
		STATUS_TIMEOUT = 3
		STATUS_NO_ROOM = 4
		STATUS_CRC_WRONG = 5
		STATUS_MIFARE_NACK = 6


class MFRC522:
		NRSTPD = 22
		MAX_LEN = 16
		PCD_IDLE = 0x00
		PCD_AUTHENT = 0x0E
		PCD_RECEIVE = 0x08
		PCD_TRANSMIT = 0x04
		PCD_TRANSCEIVE = 0x0C
		PCD_RESETPHASE = 0x0F
		PCD_CALCCRC = 0x03
		PICC_REQIDL = 0x26
		PICC_REQALL = 0x52
		PICC_ANTICOLL = 0x93
		PICC_SElECTTAG = 0x93
		PICC_AUTHENT1A = 0x60
		PICC_AUTHENT1B = 0x61
		PICC_READ = 0x30
		PICC_WRITE = 0xA0
		PICC_DECREMENT = 0xC0
		PICC_INCREMENT = 0xC1
		PICC_RESTORE = 0xC2
		PICC_TRANSFER = 0xB0
		PICC_HALT = 0x50
		MI_OK = 0
		MI_NOTAGERR = 1
		MI_ERR = 2
		Reserved00 = 0x00
		CommandReg = 0x01
		CommIEnReg = 0x02
		DivlEnReg = 0x03
		CommIrqReg = 0x04
		DivIrqReg = 0x05
		ErrorReg = 0x06
		Status1Reg = 0x07
		Status2Reg = 0x08
		FIFODataReg = 0x09
		FIFOLevelReg = 0x0A
		WaterLevelReg = 0x0B
		ControlReg = 0x0C
		BitFramingReg = 0x0D
		CollReg = 0x0E
		Reserved01 = 0x0F
		Reserved10 = 0x10
		ModeReg = 0x11
		TxModeReg = 0x12
		RxModeReg = 0x13
		TxControlReg = 0x14
		TxAutoReg = 0x15
		TxSelReg = 0x16
		RxSelReg = 0x17
		RxThresholdReg = 0x18
		DemodReg = 0x19
		Reserved11 = 0x1A
		Reserved12 = 0x1B
		MifareReg = 0x1C
		Reserved13 = 0x1D
		Reserved14 = 0x1E
		SerialSpeedReg = 0x1F
		Reserved20 = 0x20
		CRCResultRegM = 0x21
		CRCResultRegH = 0x21
		CRCResultRegL = 0x22
		Reserved21 = 0x23
		ModWidthReg = 0x24
		Reserved22 = 0x25
		RFCfgReg = 0x26
		GsNReg = 0x27
		CWGsPReg = 0x28
		ModGsPReg = 0x29
		TModeReg = 0x2A
		TPrescalerReg = 0x2B
		TReloadRegH = 0x2C
		TReloadRegL = 0x2D
		TCounterValueRegH = 0x2E
		TCounterValueRegL = 0x2F
		Reserved30 = 0x30
		TestSel1Reg = 0x31
		TestSel2Reg = 0x32
		TestPinEnReg = 0x33
		TestPinValueReg = 0x34
		TestBusReg = 0x35
		AutoTestReg = 0x36
		VersionReg = 0x37
		AnalogTestReg = 0x38
		TestDAC1Reg = 0x39
		TestDAC2Reg = 0x3A
		TestADCReg = 0x3B
		Reserved31 = 0x3C
		Reserved32 = 0x3D
		Reserved33 = 0x3E
		Reserved34 = 0x3F
		serNum = []
		# def __init__(self, chip_select_pin):
		# 		self.NRSTPD = chip_select_pin
		# 		GPIO.setmode(GPIO.BCM)
		# 		GPIO.setup(self.NRSTPD, GPIO.OUT)
		# 		self.spi = spidev.SpiDev()
		# 		self.spi.open(0, 0)
		# 		self.spi.max_speed_hz = 1000000
		def __init__(self, dev='/dev/spidev0.0', spd=1000000):
			self.dev_dictionary = spi.openSPI(device=dev,speed=spd)
			GPIO.setwarnings(False)
			GPIO.setmode(GPIO.BOARD)
			GPIO.setup(self.NRSTPD, GPIO.OUT)
			GPIO.output(self.NRSTPD, 1)
			self.MFRC522_Init()

		def MFRC522_Reset(self):
			self.MFRC522_Write(self.CommandReg, self.PCD_RESETPHASE)


		def PCD_WriteRegister(self, reg, value):
				GPIO.output(self.NRSTPD, GPIO.LOW)
				self.spi.xfer([reg, value])
				GPIO.output(self.NRSTPD, GPIO.HIGH)

		def PCD_ReadRegister(self, reg, count, values, rxAlign=0):
				if count == 0:
						return
				address = 0x80 | reg
				index = 0
				GPIO.output(self.NRSTPD, GPIO.LOW)
				self.spi.xfer([address])
				if rxAlign:
						mask = (0xFF << rxAlign) & 0xFF
						value = self.spi.xfer([address])[0]
						values[0] = (values[0] & ~mask) | (value & mask)
						index += 1
				while index < count:
						values[index] = self.spi.xfer([address])[0]
						index += 1
				values[index] = self.spi.xfer([0])[0]
				GPIO.output(self.NRSTPD, GPIO.HIGH)

		def PCD_SetRegisterBitMask(self, reg, mask):
				tmp = self.PCD_ReadRegister(reg)
				self.PCD_WriteRegister(reg, tmp | mask)

		def PCD_ClearRegisterBitMask(self, reg, mask):
				tmp = self.PCD_ReadRegister(reg)
				self.PCD_WriteRegister(reg, tmp & (~mask))
		def PCD_TransceiveData(self, sendData, sendLen, backData=None, backLen=None, validBits=None, rxAlign=0, checkCRC=False):
				waitIRq = 0x30
				return self.PCD_CommunicateWithPICC(self.PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC)

		def PCD_CommunicateWithPICC(self, command, waitIRq, sendData, sendLen, backData=None, backLen=None, validBits=None, rxAlign=0, checkCRC=False):
				txLastBits = validBits if validBits else 0
				bitFraming = (rxAlign << 4) + txLastBits

				self.PCD_WriteRegister(self.CommandReg, self.PCD_Idle)
				self.PCD_WriteRegister(self.ComIrqReg, 0x7F)
				self.PCD_WriteRegister(self.FIFOLevelReg, 0x80)
				self.PCD_WriteRegister(self.FIFODataReg, sendLen, sendData)
				self.PCD_WriteRegister(self.BitFramingReg, bitFraming)
				self.PCD_WriteRegister(self.CommandReg, command)

				if command == self.PCD_Transceive:
						self.PCD_SetRegisterBitMask(self.BitFramingReg, 0x80)

				deadline = time.time() + 0.089

				while time.time() < deadline:
					n = self.PCD_ReadRegister(self.ComIrqReg)
					if n & waitIRq:
							completed = True
							break
					if n & 0x01:
							return self.StatusCode.STATUS_TIMEOUT

				if not completed:
						return self.StatusCode.STATUS_TIMEOUT

				errorRegValue = self.PCD_ReadRegister(self.ErrorReg)
				if errorRegValue & 0x13:
						return self.StatusCode.STATUS_ERROR

				_validBits = 0

				if backData and backLen:
						n = self.PCD_ReadRegister(self.FIFOLevelReg)
						if n > backLen:
								return self.StatusCode.STATUS_NO_ROOM
						backLen = n
						self.PCD_ReadRegister(self.FIFODataReg, n, backData, rxAlign)
						_validBits = self.PCD_ReadRegister(self.ControlReg) & 0x07
						if validBits:
								validBits = _validBits

				if errorRegValue & 0x08:
						return self.StatusCode.STATUS_COLLISION

				if backData and backLen and checkCRC:
						if backLen == 1 and _validBits == 4:
								return self.StatusCode.STATUS_MIFARE_NACK
						if backLen < 2 or _validBits != 0:
								return self.StatusCode.STATUS_CRC_WRONG
						controlBuffer = [0, 0]
						status = self.PCD_CalculateCRC(backData[:-2], controlBuffer)
						if status != self.StatusCode.STATUS_OK:
								return status
						if (backData[-2] != controlBuffer[0]) or (backData[-1] != controlBuffer[1]):
								return self.StatusCode.STATUS_CRC_WRONG

				return self.StatusCode.STATUS_OK
		def PCD_CalculateCRC(self, data, length, result):
				self.PCD_WriteRegister(CommandReg, PCD_Idle)
				self.PCD_WriteRegister(DivIrqReg, 0x04)
				self.PCD_WriteRegister(FIFOLevelReg, 0x80)
				self.PCD_WriteRegister(FIFODataReg, length, data)
				self.PCD_WriteRegister(CommandReg, PCD_CalcCRC)

				deadline = time.time() + 0.089

				while time.time() < deadline:
						n = self.PCD_ReadRegister(DivIrqReg)
						if n & 0x04:
								self.PCD_WriteRegister(CommandReg, PCD_Idle)
								result[0] = self.PCD_ReadRegister(CRCResultRegL)
								result[1] = self.PCD_ReadRegister(CRCResultRegH)
								return StatusCode.STATUS_OK
						time.sleep(0.001)

				return StatusCode.STATUS_TIMEOUT
		def MIFARE_OpenUidBackdoor(self, logErrors):
			# Magic sequence:
			# > 50 00 57 CD (HALT + CRC)
			# > 40 (7 bits only)
			# < A (4 bits only)
			# > 43
			# < A (4 bits only)
			# Then you can write to sector 0 without authenticating

			self.PICC_HaltA()  # 50 00 57 CD

			cmd = 0x40
			validBits = 7  # Our command is only 7 bits. After receiving card response, this will contain amount of valid response bits.
			response = [0] * 32  # Card's response is written here
			received = len(response)
			status, received, validBits = self.PCD_TransceiveData([cmd], 1, response, received, validBits, 0, False)  # 40

			if status != self.STATUS_OK:
					if logErrors:
							print("Card did not respond to 0x40 after HALT command. Are you sure it is a UID changeable )one?")
							print("Error name: ", self.GetStatusCodeName(status))
					return False

			if received != 1 or response[0] != 0x0A:
					if logErrors:
							print("Got bad response on backdoor 0x40 command: ", hex(response[0]), " (", validBits, " )valid bits)")
					return False

			cmd = 0x43
			validBits = 8
			status, received, validBits = self.PCD_TransceiveData([cmd], 1, response, received, validBits, 0, False)  # 43

			if status != self.STATUS_OK:
					if logErrors:
							print("Error in communication at command 0x43, after successfully executing 0x40")
							print("Error name: ", self.GetStatusCodeName(status))
					return False

			if received != 1 or response[0] != 0x0A:
					if logErrors:
							print("Got bad response on backdoor 0x43 command: ", hex(response[0]), " (", validBits, " )valid bits)")
					return False

			# You can now write to sector 0 without authenticating!
			return True

		def PICC_HaltA(self):
			# Build command buffer
			buffer = [0] * 4
			buffer[0] = self.PICC_HALT
			buffer[1] = 0

			# Calculate CRC_A
			result = [0] * 2
			status, result[0], result[1] = self.PCD_CalculateCRC(buffer[:2], len(buffer[:2]), result)

			if status != self.STATUS_OK:
					return status

			# Send the command.
			status, _, _ = self.PCD_TransceiveData(buffer, len(buffer), None, 0)

			if status == self.STATUS_TIMEOUT:
					return self.STATUS_OK
			if status == self.STATUS_OK:
					return self.STATUS_ERROR

			return status

		def MFRC522_Write(self, blockAddr, writeData):
			buff = []
			buff.append(self.PICC_WRITE)
			buff.append(blockAddr)
			crc = self.CalulateCRC(buff)
			buff.append(crc[0])
			buff.append(crc[1])
			(status, backData, backLen) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, buff)
			if not(status == self.MI_OK) or not(backLen == 4) or not((backData[0] & 0x0F) == 0x0A):
					status = self.MI_ERR

			print( "%s backdata &0x0F == 0x0A %s" % (backLen, backData[0]&0x0F))
			if status == self.MI_OK:
					i = 0
					buf = []
					while i < 16:
							buf.append(writeData[i])
							i = i + 1
					crc = self.CalulateCRC(buf)
					buf.append(crc[0])
					buf.append(crc[1])
					(status, backData, backLen) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE,buf)
					if not(status == self.MI_OK) or not(backLen == 4) or not((backData[0] & 0x0F) == 0x0A):
							print( "Error while writing")
					if status == self.MI_OK:
							print( "Data written")

		def MFRC522_Reset(self):
			self.Write_MFRC522(self.CommandReg, self.PCD_RESETPHASE)

		def AntennaOn(self):
			temp = self.Read_MFRC522(self.TxControlReg)
			if(~(temp & 0x03)):
				self.SetBitMask(self.TxControlReg, 0x03)


		def Write_MFRC522(self, addr, val):
			spi.transfer(self.dev_dictionary, ((addr<<1)&0x7E,val))

		def SetBitMask(self, reg, mask):
				tmp = self.Read_MFRC522(reg)
				self.Write_MFRC522(reg, tmp | mask)

		def ClearBitMask(self, reg, mask):
			tmp = self.Read_MFRC522(reg);
			self.Write_MFRC522(reg, tmp & (~mask))

		def Read_MFRC522(self, addr):
			val = spi.transfer(self.dev_dictionary, (((addr<<1)&0x7E) | 0x80,0))
			return val[1]

		def MFRC522_Init(self):
				GPIO.output(self.NRSTPD, 1)

				self.MFRC522_Reset()


				self.Write_MFRC522(self.TModeReg, 0x8D)
				self.Write_MFRC522(self.TPrescalerReg, 0x3E)
				self.Write_MFRC522(self.TReloadRegL, 30)
				self.Write_MFRC522(self.TReloadRegH, 0)

				self.Write_MFRC522(self.TxAutoReg, 0x40)
				self.Write_MFRC522(self.ModeReg, 0x3D)
				self.AntennaOn()



if __name__ == '__main__':
		MIFAREReader = MFRC522()
		MIFAREReader.MIFARE_OpenUidBackdoor(True)
