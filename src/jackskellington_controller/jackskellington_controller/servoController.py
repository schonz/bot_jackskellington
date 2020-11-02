import rclpy
from rclpy.node import Node
from smbus2 import SMBus
from std_msgs.msg import String
import time


# == Constants ==
CH_I2C = 1  # RPi calls i2c ch 1
# PCA9685
PCA_I2C_ADDR = 0x40  #0xE0
PCA_ADDR_MODE1 = 0x00
PCA_ADDR_MODE2 = 0x01
PCA_ADDR_PRESCALE = 0xFE
PCA_ADDR_READ = 0xE1   # 1110 0001
PCA_ADDR_WRITE = 0xE0  # 1110 0000
PCA_PULSE_LEN = 4096
PCA_OSC = 25000000  # Hz or 0.04us
PCA_ADDR_CH = [
    0x06,  # 0
    0x0A  # 1
]
# Servo HS-300
HS300_MAX_PULSE = 2100  #us
HS300_MIN_PULSE = 900  #us
HS300_FREQ = 50 #Hz or 20ms, 20000us


class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self.subscription = self.create_subscription(
            String,
            'servo_commands',
            self.listener_callback,
            10
        )

        self.pulseStart = 500  #HS300_MAX_PULSE - HS300_MIN_PULSE

        # Connect to i2c servo controller
        #self.bus = SMBus(CH_I2C)
        self.initPCA9685()
        self.setServo(0, 50)


    def getPrescaleValue(self, targetFreq):
        prescale = round(PCA_OSC / (PCA_PULSE_LEN * targetFreq) - 1)
        return prescale

    def initPCA9685(self):
        # Set Mode (Turn on Device)
        
        # Set all channels off
        self.writeI2C(PCA_I2C_ADDR, 0xFA, [0x00, 0x00, 0x00, 0x00])
        
        # reset
        #with SMBus(1) as bus:
        #    bus.write_byte(0x00,0x06)
        #time.sleep(0.5)

        # start in sleep so we can set prescale on oscillator
        # -- Set Mode 1 --
        self.pcaMode1 = self.readI2C(PCA_I2C_ADDR, PCA_ADDR_MODE1, 1)[0]
        self.get_logger().info('orig mode1: {0}'.format(hex(self.pcaMode1)))
        self.pcaMode1 = 0x31  # 00110001
        self.pcaMode1 = self.pcaMode1 | 0x01  # AllCall
        self.pcaMode1 = self.pcaMode1 | 0x10  # Sleep
        #self.pcaMode1 = self.pcaMode1 | 0x40  # Internal clock
        self.writeI2C(PCA_I2C_ADDR, PCA_ADDR_MODE1, [self.pcaMode1])
        self.get_logger().info('new mode1: {0}'.format(hex(self.pcaMode1)))
        self.get_logger().info('{0}'.format(hex(self.readI2C(PCA_I2C_ADDR, PCA_ADDR_MODE1, 1)[0])))
        # -- Set Mode2 --
        self.pcaMode2 = self.readI2C(PCA_I2C_ADDR, PCA_ADDR_MODE2, 1)[0]
        # turn on totems
        self.pcaMode2 = self.pcaMode2 | 0x04
        self.writeI2C(PCA_I2C_ADDR, PCA_ADDR_MODE2, [self.pcaMode2])
        
        # == Set Prescaler ==
        # Set the prescaler so our channels are at 50Hz
        prescaleVal = self.getPrescaleValue(HS300_FREQ)
        self.writeI2C(PCA_I2C_ADDR, PCA_ADDR_PRESCALE, [prescaleVal])
        self.get_logger().info('prescale: {0}'.format(hex(prescaleVal)))
        time.sleep(0.5)
        self.get_logger().info('{0}'.format(hex(self.readI2C(PCA_I2C_ADDR, PCA_ADDR_PRESCALE, 1)[0])))
        
        # == wrap up ==
        self.get_logger().info('wrap-up')
        self.pcaMode1 = self.readI2C(PCA_I2C_ADDR, PCA_ADDR_MODE1, 1)[0]
        self.get_logger().info('change? {0}'.format(hex(self.pcaMode1)))
        # Turn Device back on
        self.pcaMode1 = self.pcaMode1 & ~0x10
        self.writeI2C(PCA_I2C_ADDR, PCA_ADDR_MODE1, [self.pcaMode1])
        self.get_logger().info('turn on: {0}'.format(hex(self.pcaMode1)))
        time.sleep(0.5)
        # Restart PWM channels
        self.pcaMode1 = self.pcaMode1 | 0x80
        self.writeI2C(PCA_I2C_ADDR, PCA_ADDR_MODE1, [self.pcaMode1])
        self.get_logger().info('restart: {0}'.format(hex(self.pcaMode1)))
        time.sleep(0.5)
        self.get_logger().info('init done')


    """
    Listens to ROS topic
    msg: String, format 'channel':'target position as %'
        (e.g. '0:100,1:50,2:25')
    """
    def listener_callback(self, msg):
        self.get_logger().info('servo cmd received: %s' % msg.data)

        cmds = msg.data.split(',')
        for cmd in cmds:
            cmd_args = cmd.split(':')

            # check integrety
            if len(cmd_args) != 2:
                self.get_logger().info('bad command')
                continue

            # set command
            self.get_logger().info('setting ch{0} to {1}%'.format(cmd_args[0], cmd_args[1]))
            self.setServo(int(cmd_args[0]), int(cmd_args[1]))

    def setServo(self, ch_servo, position):
        # Convert position (%) to pulse time
        pulseLen = int((HS300_MAX_PULSE - HS300_MIN_PULSE) * position / 100)
        pulseLen = int(360 * position / 100)
        # off time (on time we keep the same)
        pulseEnd = self.pulseStart + pulseLen

        # convert to bits
        byte_on_L = self.pulseStart & 0xFF
        byte_on_H = (self.pulseStart >> 8) & 0xFF
        byte_off_L = pulseEnd & 0xFF
        byte_off_H = (pulseEnd >> 8) & 0xFF
        
        self.get_logger().info('pulse start %d' % self.pulseStart)
        self.get_logger().info('pulse end %d' % pulseEnd)

        values = [
            byte_on_L,
            byte_on_H,
            byte_off_L,
            byte_off_H
        ]

        self.writeI2C(PCA_I2C_ADDR, PCA_ADDR_CH[ch_servo], values)
        #chData = self.readI2C(PCA_I2C_ADDR, PCA_ADDR_CH[ch_servo], 4)
        #for x in chData:
        #    self.get_logger().info('byte: {0}'.format(x))

    def writeI2C(self, i2cAddr, register, values):
        with SMBus(CH_I2C) as bus:
            bus.write_i2c_block_data(i2cAddr, register, values)

    def readI2C(self, i2cAddr, register, nBytes):
        msg = None
        with SMBus(CH_I2C) as bus:
            msg = bus.read_i2c_block_data(i2cAddr, register, nBytes)
        return msg


def main(args=None):
    rclpy.init(args=args)

    servo_controller = ServoController()

    rclpy.spin(servo_controller)

    # == Clean Up ==
    servo_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
