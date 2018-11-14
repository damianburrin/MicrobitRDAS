######################
#    ---11/11/18---   #
# Microbit Altimeter #
#     v1.04(BETA)    #
#      D Burrin      #
######################
#imports
import math
from microbit import uart,display,accelerometer,sleep,pin0,pin1,pin19,pin20,i2c,button_a,button_b,reset
import time
#Author: shaoziyang
#Date:   2018.2
BMP280_I2C_ADDR = 0x76                                                                                          
class BMP280():
	def __init__(self):
		self.dig_T1 = self.get2Reg(0x88)
		self.dig_T2 = self.short(self.get2Reg(0x8A))
		self.dig_T3 = self.short(self.get2Reg(0x8C))
		self.dig_P1 = self.get2Reg(0x8E)
		self.dig_P2 = self.short(self.get2Reg(0x90))
		self.dig_P3 = self.short(self.get2Reg(0x92))
		self.dig_P4 = self.short(self.get2Reg(0x94))
		self.dig_P5 = self.short(self.get2Reg(0x96))
		self.dig_P6 = self.short(self.get2Reg(0x98))
		self.dig_P7 = self.short(self.get2Reg(0x9A))
		self.dig_P8 = self.short(self.get2Reg(0x9C))
		self.dig_P9 = self.short(self.get2Reg(0x9E))
		self.setReg(0xF4, 0x2F)
		self.setReg(0xF5, 0x0C)
		self.T = 0
		self.P = 0
		self.version = '1.0'
	def	short(self,	dat):
		if dat > 32767:
			return dat - 65536
		else:
			return dat

	# set reg
	def	setReg(self, reg, dat):
		buf	= bytearray(2)
		buf[0] = reg
		buf[1] = dat
		i2c.write(BMP280_I2C_ADDR, buf)
	# get reg
	def	getReg(self, reg):
		buf	= bytearray(1)
		buf[0] = reg
		i2c.write(BMP280_I2C_ADDR, buf)
		t =	i2c.read(BMP280_I2C_ADDR, 1)
		return t[0]
	# get two reg
	def	get2Reg(self, reg):
		buf	= bytearray(1)
		buf[0] = reg
		i2c.write(BMP280_I2C_ADDR, buf)
		t =	i2c.read(BMP280_I2C_ADDR, 2)
		return t[1]*256	+ t[0]

	def get(self):
		adc_T = (self.getReg(0xFA)<<12) + (self.getReg(0xFB)<<4) + (self.getReg(0xFC)>>4)
		var1 = (((adc_T>>3)-(self.dig_T1<<1))*self.dig_T2)>>11
		var2 = (((((adc_T>>4)-self.dig_T1)*((adc_T>>4) - self.dig_T1))>>12)*self.dig_T3)>>14
		t = var1+var2
		self.T = ((t * 5 + 128) >> 8)/100
		var1 = (t>1) - 64000
		var2 = (((var1>>2) * (var1>>2)) >> 11 ) * self.dig_P6
		var2 = var2 + ((var1*self.dig_P5)<<1)
		var2 = (var2>>2)+(self.dig_P4<<16)
		var1 = (((self.dig_P3*((var1>>2)*(var1>>2))>>13)>>3) + (((self.dig_P2) * var1)>>1))>>18
		var1 = ((32768+var1)*self.dig_P1)>>15
		if var1 == 0:
			return  # avoid exception caused by division by zero
		adc_P = (self.getReg(0xF7)<<12) + (self.getReg(0xF8)<<4) + (self.getReg(0xF9)>>4)
		p=((1048576-adc_P)-(var2>>12))*3125
		if p < 0x80000000:
			p = (p << 1) // var1
		else:
			p = (p // var1) * 2
		var1 = (self.dig_P9 * (((p>>3)*(p>>3))>>13))>>12
		var2 = (((p>>2)) * self.dig_P8)>>13
		self.P = p + ((var1 + var2 + self.dig_P7) >> 4)
		return [self.T, self.P]

	# get Temperature in Celsius
	def getTemp(self):
		self.get()
		return self.T

	# get Pressure in Pa
	def getPress(self):
		self.get()
		return (self.P)

	# Calculating absolute altitude
	def	getAltitude(self,Pressure):
		return (44330*(1-(self.getPress()/Pressure)**(1/5.255)))
        
 	# get Pressure in Pa for ground level
	def SLP(self):
		self.get()
		return self.P

        

	# sleep mode
	def poweroff(self):
		self.setReg(0xF4, 0)

	# normal mode
	def poweron(self):
		self.setReg(0xF4, 0x2F)

###My Functions###
##calc 3d acceleration###
def get_acceleration():
    y = accelerometer.get_y()
    z = accelerometer.get_z() 
    x = accelerometer.get_x()
    #Calculate 3D Acceleration
    acceleration = math.sqrt(x**2 + y**2 + z**2)
    return(acceleration)
 
#Create headers for CSV 
def write_headers():
    uart.init(baudrate=9600, bits=8, parity=None, stop=1,tx=pin0,rx=pin1)
    uart.write("\nTime,X,Y,Z,Acceleration,Pressure,Altitude,Temperature\n\n")
    #uart.write("\nTime,Acceleration,Pressure,Altitude,Temperature\n\n")
    
    uart.init(baudrate=9600, bits=8, parity=None, stop=1,tx=None,rx=None)

###main code###
#sleep loop#
while get_acceleration() < 1500:
    sleep(50)
    display.show("X")
#write header CSV
write_headers()
display.show("L")
bmp280 = BMP280()
#open UART
uart.init(baudrate=9600, bits=8, parity=None, stop=1,tx=pin0,rx=pin1)
#get start time
now =time.ticks_ms()
while True:
    #log data
    timer= round( ((time.ticks_ms()-now)/1000),2)
    uart.write(str(timer))
    uart.write(",")
    uart.write(str(accelerometer.get_x()))#x
    uart.write(",")
    uart.write(str(accelerometer.get_y()))#y
    uart.write(",")
    uart.write(str(accelerometer.get_z()))#z
    uart.write(",")
    acceleration = get_acceleration()
    uart.write(str(acceleration))
    uart.write(",")
    pressure=bmp280.getPress()
    uart.write(str(pressure/100))#converted to Pa
    uart.write(",")
    altitude=bmp280.getAltitude(pressure)
    uart.write(str(altitude))#in meters above start point
    uart.write(",")
    Ctemp=bmp280.getTemp()
    uart.write(str(Ctemp))
    uart.write("\n")
        
    sleep(15)
    #stop logging on button b
    if button_b.is_pressed():
	#wait for 0.5 seconds to ensure last value is written befofe exit
        sleep(500)
        uart.init(baudrate=9600, bits=8, parity=None, stop=1,tx=None,rx=None)
        #Clear the screen
        display.clear()
        display.show("E")
       
        #break
	sleep(2000)
	#use reset to send back to main screen afer 2 seconds
	reset()
        
    if button_a.is_pressed():
        sleep(200)
        reset()
