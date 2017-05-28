import sys
import serial
import argparse
import struct

commands = {
	'BBIO1': '\x00',
	'SPI1': '\x01',
	'I2C1': '\x02',
	'ART1': '\x03',
	'1WO1': '\x04',
	'RAW1': '\x05',
	'RESET': '\x0F',
	'STEST': '\x10',
}

def arg_auto_int(x):
	return int(x, 0)

class FatalError(RuntimeError):
	def __init__(self, message):
		RuntimeError.__init__(self, message)

def main():
	parser = argparse.ArgumentParser(description = 'Bus Pirate binary interface demo', prog='binaryModeDemo')

	parser.add_argument(
		'--port', '-p',
		help = 'Serial port device',
		default = '/dev/ttyUSB0')

	parser.add_argument(
		'--baud', '-b',
		help = 'Serial port baud rate',
		type = arg_auto_int,
		default = 115200)

	args = parser.parse_args()

	print '\nTrying port: ', args.port, ' at baudrate: ', args.baud

	try:
		port = serial.Serial(args.port, args.baud, timeout=0.1)
	except Exception as e:
		print 'I/O error({0}): {1}'.format(e.errno, e.strerror)
		print 'Port cannot be opened'
	else:
		print 'Ready!'
		print 'Setting up mode\n'

		'''count = 0
		done = False
		while count < 20 and not done:
			count += 1
			port.write(commands.get('BBIO1'))
			got = port.read(5)
			if got == 'BBIO1':
				done = True
		if not done:
			port.close()
			raise FatalError('Buspirate failed to enter binary mode')
'''
		print 'Resetting...\n'
		port.write(str('#\n').encode('ascii'))
		while True:
			got = port.readline()
			if not got:
				break
			print(got)
		
		# Switch modes
		port.write(str('m\n').encode('ascii'))
		lines = port.readlines()
		# Go to UART
		port.write(str('3\n').encode('ascii'))
		lines = port.readlines()
		# Select baudrate (9600)
		port.write(str('5\n').encode('ascii'))
		lines = port.readlines()
		# Data bits and parity (8 bits, 
		port.write(str('1\n').encode('ascii'))
		lines = port.readlines()
		# Stop bits
		port.write(str('1\n').encode('ascii'))
		lines = port.readlines()
		# Receive polarity
		port.write(str('1\n').encode('ascii'))
		lines = port.readlines()
		# Output type selection 
		port.write(str('1\n').encode('ascii'))
		lines = port.readlines()
		# PSU On
		port.write(str('W\n').encode('ascii'))
		lines = port.readlines()

		port.write(str('(3)\ny').encode('ascii'))
		lines = port.readlines()

		print 'Reading...\n'

		# Read data here
		while True:
			line = port.readline()
			if line:
				if line[0] == 'A':
#time = ((ord(line[1]) << 8) | ord(line[2]))
#latitude = ((ord(line[1]) << 24) | (ord(line[2]) << 16) | (ord(line[3]) << 8) | (ord(line[4]) << 0))
#					b = ''.join(i for i in line[1:5])
					status = ord(line[1])
					latitude = struct.pack('4B', ord(line[2]), ord(line[3]), ord(line[4]), ord(line[5]))
					longitude = struct.pack('4B', ord(line[6]), ord(line[7]), ord(line[8]), ord(line[9]))
					
#					longitude = ((ord(line[7]) << 24) | (ord(line[8]) << 16) | (ord(line[9]) << 8) | (ord(line[10])))
#					print('Time: ' + str(time))
					print('Status: ' + str(status))
					print('Latitude: ' + str(struct.unpack('>f', latitude)[0]))
					print('Longitude: ' + str(struct.unpack('>f', longitude)[0]))
				
		
		# PSU Off
		#port.write(str('w\n').encode('ascii'))
		#lines = port.readlines()

		port.close()

if __name__ == '__main__':
	try:
		main()
	except FatalError as e:
		print '\nA fatal error occurred: %s' % e
		sys.exit(2)

