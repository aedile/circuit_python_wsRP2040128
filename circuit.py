# This is an implementation of the Waveshare 1.28" LCD RP2040 Board
# https://www.waveshare.com/wiki/1.28inch_LCD_RP2040_Board
# It uses the CircuitPython libraries and the GC9A01 display driver
# https://circuitpython.readthedocs.io/projects/gc9a01/en/latest/
# I had to implement my own battery and IMU classes because the
# existing ones didn't work with CircuitPython.  
# @author: Jesse R. Castro
# @TODO: Create a separate class for animations

import random
import time

import board
import busio
import displayio
import gc9a01
import terminalio
import analogio

from adafruit_display_text import label
from adafruit_bitmap_font import bitmap_font

# Note: this is important during dev or else you can't use the screen
# for the actual app, it tries to default to debugging.
displayio.release_displays()

# A generic class to describe battery status
class Battery(object):
    # Initialize the battery
    # returns: nothing
    def __init__(self, pin=board.BAT_ADC):
        self._pin = analogio.AnalogIn(pin)
        self._max_voltage = 4.14
        self._min_voltage = 3.4
        self._max_diff = self._max_voltage - self._min_voltage
        self._diff = 0.0
        self._voltage = 0.0
        self._percent = 0.0
        self._charging = False
        self._discharging = False
        self._full = False
        self._empty = False
        self._update()

    # Update the battery status
    # returns: nothing
    def _update(self):
        # Read the battery voltage
        self._voltage = self._pin.value * 3.3 / 65535 * 2
        self._diff = self._max_voltage - self._voltage
        # Convert the voltage to a percentage
        if self._voltage > self._max_voltage:
            self._percent = 100.0
        elif self._voltage < self._min_voltage:
            self._percent = 0.0
        else:
            self._percent = (self._diff / self._max_diff) * 100.0 
        # Determine the charging status
        if self._voltage > 4.14:
            self._charging = True
            self._discharging = False
            self._full = True
            self._empty = False
        elif self._voltage < 3.4:
            self._charging = False
            self._discharging = False
            self._full = False
            self._empty = True
        else:
            self._charging = False
            self._discharging = True
            self._full = False
            self._empty = False

    # Get the battery voltage
    # returns: the battery voltage
    @property
    def voltage(self):
        self._update()
        return self._voltage

    # Get the battery percentage
    # returns: the battery percentage
    @property
    def percent(self):
        self._update()
        return self._percent

    # Get the charging status
    # returns: True if the battery is charging, False otherwise
    @property
    def charging(self):
        self._update()
        return self._charging

    # Get the discharging status
    # returns: True if the battery is discharging, False otherwise
    @property
    def discharging(self):
        self._update()
        return self._discharging

# A class to interface with a Microchip QMI8658 6-axis IMU
# https://www.microchip.com/wwwproducts/en/QMI8658
# Working code already exists in micropython, this is an 
# adaptation of that code for use with CircuitPython (see
# micro.py for the original code)
class QMI8658_Accelerometer(object):
    # Initialize the hardware
    # address: the I2C address of the device
    # returns: nothing
    def __init__(self,address=0X6B, scl=board.GP7, sda=board.GP6):
        self._address = address
        self._bus = busio.I2C(scl,sda)
        if self.who_am_i():
            self.rev = self.read_revision()
        else:
            raise Exception("QMI8658 not found")
        self.config_apply()
    
    # Read a byte from the specified register
    # register: the register to read from
    # returns: the byte read
    def _read_byte(self,register):
        return self._read_block(register,1)[0]

    # Read a block of bytes from the specified register
    # register: the register to begin the read from
    # length: the number of bytes to read
    # returns: a list of bytes read
    def _read_block(self, register, length=1):
        while not self._bus.try_lock():
            pass
        try:
            rx = bytearray(length)
            self._bus.writeto(self._address, bytes([register]))
            self._bus.readfrom_into(self._address, rx)
        finally:
            self._bus.unlock()    
        return rx
    
    # Read a 16-bit unsigned integer from the specified register
    # register: the register to begin the read from
    # returns: the 16-bit unsigned integer read
    def _read_u16(self,register):
        return (self._read_byte(register) << 8) + self._read_byte(register+1)

    # Write a byte to the specified register
    # register: the register to write to
    # value: the byte to write
    # returns: nothing    
    def _write_byte(self,register,value):
        while not self._bus.try_lock():
            pass
        try:
            self._bus.writeto(self._address, bytes([register, value]))
            #self._bus.writeto(self._address, bytes([value]))
        finally:
            self._bus.unlock()

    # Make sure this device is what it thinks it is
    # returns: True if the device is what it thinks it is, False otherwise  
    def who_am_i(self):
        bRet=False
        rec = self._read_byte(0x00)
        if (0x05) == rec:
            bRet = True   
        return bRet

    # Read the revision of the device
    # returns: the revision of the device
    def read_revision(self):
        return self._read_byte(0x01)

    # Apply the configuration to the device by writing to 
    # the appropriate registers.  See device datasheet for
    # details on the configuration.
    # returns: nothing    
    def config_apply(self):
        # REG CTRL1
        self._write_byte(0x02,0x60)
        # REG CTRL2 : QMI8658AccRange_8g  and QMI8658AccOdr_1000Hz
        self._write_byte(0x03,0x23)
        # REG CTRL3 : QMI8658GyrRange_512dps and QMI8658GyrOdr_1000Hz
        self._write_byte(0x04,0x53)
        # REG CTRL4 : No
        self._write_byte(0x05,0x00)
        # REG CTRL5 : Enable Gyroscope And Accelerometer Low-Pass Filter 
        self._write_byte(0x06,0x11)
        # REG CTRL6 : Disables Motion on Demand.
        self._write_byte(0x07,0x00)
        # REG CTRL7 : Enable Gyroscope And Accelerometer
        self._write_byte(0x08,0x03)

    # Read the raw accelerometer and gyroscope data from the device
    # returns: a list of 6 integers, the first 3 are the accelerometer
    #          data, the last 3 are the gyroscope data
    def read_raw_xyz(self):
        xyz=[0,0,0,0,0,0]
        raw_timestamp = self._read_block(0x30,3)
        raw_acc_xyz=self._read_block(0x35,6)
        raw_gyro_xyz=self._read_block(0x3b,6)
        raw_xyz=self._read_block(0x35,12)
        timestamp = (raw_timestamp[2]<<16)|(raw_timestamp[1]<<8)|(raw_timestamp[0])
        for i in range(6):
            # xyz[i]=(raw_acc_xyz[(i*2)+1]<<8)|(raw_acc_xyz[i*2])
            # xyz[i+3]=(raw_gyro_xyz[((i+3)*2)+1]<<8)|(raw_gyro_xyz[(i+3)*2])
            xyz[i] = (raw_xyz[(i*2)+1]<<8)|(raw_xyz[i*2])
            if xyz[i] >= 32767:
                xyz[i] = xyz[i]-65535
        return xyz

    # Read the accelerometer and gyroscope data from the device and return
    # in human-readable format.
    # returns: a list of 6 floats, the first 3 are the accelerometer
    #         data, the last 3 are the gyroscope data    
    def read_xyz(self):
        xyz=[0,0,0,0,0,0]
        raw_xyz=self.read_raw_xyz()  
        #QMI8658AccRange_8g
        acc_lsb_div=(1<<12)
        #QMI8658GyrRange_512dps
        gyro_lsb_div = 64
        for i in range(3):
            xyz[i]=raw_xyz[i]/acc_lsb_div#(acc_lsb_div/1000.0)
            xyz[i+3]=raw_xyz[i+3]*1.0/gyro_lsb_div
        return xyz

# A class to interface with a GC9A01 round display
# This class is based on the Adafruit CircuitPython GC9A01 library
# It is modified to work with the WS RP2040 1.28" dev board and
# helper classes have been added.
class GC9A01_Display(object):
    # Initialize the display
    # autoshow: if True, the display will be updated after each draw operation
    #           if False, the display will not be updated until show() is called
    #           default is True
    # returns: nothing
    def __init__(self, autoshow=True):
        # Pins
        self.clk = board.GP10
        self.mosi = board.GP11
        self.rst = board.GP12
        self.dc = board.GP8
        self.cs = board.GP9
        self.bl = board.GP25
        
        self.auto_show = autoshow

        self.width = 240
        self.height = 240
        
        self.spi = busio.SPI(clock=self.clk, MOSI=self.mosi)
        self.display_bus = displayio.FourWire(self.spi, command=self.dc, chip_select=self.cs, reset=self.rst)
        self.display = gc9a01.GC9A01(self.display_bus, width=self.width, height=self.height, backlight_pin=self.bl)

        self.group = displayio.Group()

    # Draw a rectangle on the display
    # x: the x coordinate of the upper left corner of the rectangle
    # y: the y coordinate of the upper left corner of the rectangle
    # w: the width of the rectangle
    # h: the height of the rectangle
    # color: the color of the rectangle
    # returns: the displayio.TileGrid object that was created
    def draw_rectangle(self,x,y,w,h,color):
        bitmap = displayio.Bitmap(w, h, 1)
        palette = displayio.Palette(1)
        palette[0] = color
        tile_grid = displayio.TileGrid(bitmap, pixel_shader=palette, x=x, y=y)
        self.group.append(tile_grid)
        if self.auto_show:
            self.display.show(self.group)
        return tile_grid
    
    # Draw text on the display
    # x: the x coordinate of the upper left corner of the text
    # y: the y coordinate of the upper left corner of the text
    # text: the text to display
    # color: the color of the text
    # font: the font to use
    # returns: the label.Label object that was created
    def draw_text(self,x,y,text,color,font):
        text_area = label.Label(font, text=text, color=color)
        text_area.x = x
        text_area.y = y
        self.group.append(text_area)
        if self.auto_show:
            self.display.show(self.group)
        return text_area    

    # Draw a bitmap file on the display
    # NOTE: proceed with caution as memory is low on this device
    # x: the x coordinate of the upper left corner of the bitmap
    # y: the y coordinate of the upper left corner of the bitmap
    # bitmap_path: the path to the bitmap file on the device
    # returns: the displayio.TileGrid object that was created
    def draw_bitmap(self,x,y,bitmap_path):
        return self.draw_sprite(x,y,bitmap_path,1,1,0)
    
    # Draw an animated sprite on the display
    # NOTE: proceed with caution as memory is low on this device
    # x: the x coordinate of the upper left corner of the sprite
    # y: the y coordinate of the upper left corner of the sprite
    # sprite_path: the path to the sprite file on the device
    # sprite_tiles_x: the number of tiles in the sprite in the x direction
    # sprite_tiles_y: the number of tiles in the sprite in the y direction
    # sprite_tile_width: the width of each tile in the sprite
    # sprite_tile_height: the height of each tile in the sprite
    # sprite_starting_tile: the starting tile in the sprite
    #                    default is 0
    def draw_sprite(self,x,y,sprite_path,sprite_tiles_x,sprite_tiles_y,sprite_starting_tile=0):
        bitmap = displayio.OnDiskBitmap(sprite_path)
        sprite_tile_width = bitmap.width // sprite_tiles_x
        sprite_tile_height = bitmap.height // sprite_tiles_y
        tile_grid = displayio.TileGrid(bitmap, pixel_shader=bitmap.pixel_shader, width=1, height=1, tile_width=sprite_tile_width, tile_height=sprite_tile_height, default_tile=sprite_starting_tile, x=x, y=y)
        self.group.append(tile_grid)
        if self.auto_show:
            self.display.show(self.group)
        return tile_grid

    # Fill the display with a color
    # color: the color to fill the display with
    # returns: the displayio.TileGrid object that was created
    def fill(self,color):
        return self.draw_rectangle(0,0,self.width,self.height,color)

    # Show the display
    # returns: nothing
    def show(self):
        self.display.show(self.group)
    
# CircuitPython class representing the Waveshare RP2040 1.28" 
# development board.  This class is used to initialize the
# display and the pins used to communicate with the display as 
# well as the pins used to communicate with the accelerometer.
# It also has helper functions to make it easier to draw to the
# display.
class wsRP2040128():
    # Initialize the board
    # returns: nothing
    def __init__(self):
        # Important - release the display from dev stuff
        displayio.release_displays()
        # Initialize hardware
        self._display = GC9A01_Display(True)
        self._qmi8658 =QMI8658_Accelerometer()
        # This is where we'll track our sprites
        self.sprites = {}
        # The accelerometer revision
        self.qmi8658rev = self._qmi8658.rev
        # Accelerometer data
        self.accel = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0
        }
        # Gyroscope data
        self.gyro = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0
        }
        # Battery data
        self._battery = Battery()
        self.battery_voltage = 0.0
        self.battery_percent = 0.0
        self.battery_charging = False
        self.battery_status = 'init'

        # Time tracker
        self.time = time.monotonic()
        
    # Passthrough method to draw a rectangle on the display
    # sprite_id: text identifier for the sprite
    # x: the x coordinate of the upper left corner of the rectangle
    # y: the y coordinate of the upper left corner of the rectangle
    # w: the width of the rectangle
    # h: the height of the rectangle
    # color: the color of the rectangle
    # returns: nothing    
    def draw_rectangle(self,sprite_id,x,y,w,h,color):
        self.sprites[sprite_id] = self._display.draw_rectangle(x,y,w,h,color)

    # Passthrough method to draw text on the display
    # sprite_id: text identifier for the sprite
    # x: the x coordinate of the upper left corner of the text
    # y: the y coordinate of the upper left corner of the text
    # text: the text to display
    # color: the color of the text
    # font: the font to use
    # returns: nothing
    def draw_text(self,sprite_id, x,y,text,color,font):
        self.sprites[sprite_id] = self._display.draw_text(x,y,text,color,font)
    
    # Passthrough method to draw a bitmap file on the display
    # sprite_id: text identifier for the sprite
    # x: the x coordinate of the upper left corner of the bitmap
    # y: the y coordinate of the upper left corner of the bitmap
    # bitmap_path: the path to the bitmap file on the device
    # returns: nothing
    def draw_bitmap(self,sprite_id, x,y,bitmap_path):
        self.sprites[sprite_id] = self._display.draw_bitmap(x,y,bitmap_path)

    # Passthrough method to draw an animated sprite on the display
    # sprite_id: text identifier for the sprite
    # x: the x coordinate of the upper left corner of the sprite
    # y: the y coordinate of the upper left corner of the sprite
    # sprite_path: the path to the sprite file on the device
    # sprite_tiles_x: the number of tiles in the sprite in the x direction
    # sprite_tiles_y: the number of tiles in the sprite in the y direction
    # sprite_starting_tile: the starting tile in the sprite
    #                    default is 0
    # returns: nothing
    def draw_sprite(self,sprite_id,x,y,sprite_path,sprite_tiles_x,sprite_tiles_y,sprite_starting_tile=0):
        self.sprites[sprite_id] = self._display.draw_sprite(x,y,sprite_path,sprite_tiles_x,sprite_tiles_y,sprite_starting_tile)

    # Passthrough method to fill the display with a color
    # color: the color to fill the display with
    # returns: nothing
    def fill(self,color):
        self.sprites['background'] = self._display.fill(color)

    # Helper method to convert a color string to a color value
    # colstr: the color string
    # returns: the color value
    def color(self,colstr):
        if colstr == 'red':
            return 0xFF0000
        elif colstr == 'green':
            return 0x00FF00
        elif colstr == 'blue':
            return 0x0000FF
        elif colstr == 'yellow':
            return 0xFFFF00
        elif colstr == 'cyan':
            return 0x00FFFF
        elif colstr == 'magenta':
            return 0xFF00FF
        elif colstr == 'white':
            return 0xFFFFFF
        elif colstr == 'black':
            return 0x000000
        elif colstr == 'orange':
            return 0xFFA500
        else:
            return 0x000000

    # Show the display
    # returns: nothing
    def _show(self):
        self._display.show()  
    
    # Update the accelerometer data
    # returns: nothing
    def _update_accelerometer(self):
        xyz = self._qmi8658.read_xyz()
        self.accel['x'] = xyz[0]
        self.accel['y'] = xyz[1]
        self.accel['z'] = xyz[2]
        self.gyro['x'] = xyz[3]
        self.gyro['y'] = xyz[4]
        self.gyro['z'] = xyz[5]          

    # Update the battery data 
    # returns: nothing
    def _update_battery(self):
        self.battery_voltage = self._battery.voltage
        self.battery_percent = self._battery.percent
        self.battery_charging = self._battery.charging
        if self.battery_charging:
            self.battery_status = 'chg'
        else:
            self.battery_status = '{}%'.format(self.battery_percent)

    # Update the hardware on the board for this pass
    # returns: nothing
    def update(self):
        self._update_accelerometer()
        self._show()
        self._update_battery()

    # Demo code - run in the main loop
    # counter: the current animation iteration
    # sleep_time: the time to sleep between updates
    # returns: nothing
    def demo(self, counter, sleep_time=0.01):
        try:
            # Update the sensor info
            self.sprites['volt_text'].text = "Vol: {:.2f}v".format(self.battery_voltage)
            self.sprites['charge_text'].text = "Chg: {}".format(self.battery_status)
            self.sprites['accel_x'].text = "X: {:.2f}".format(self.accel['x'])
            self.sprites['accel_y'].text = "Y: {:.2f}".format(self.accel['y'])
            self.sprites['accel_z'].text = "Z: {:.2f}".format(self.accel['z'])
            self.sprites['gyro_x'].text = "X: {:3.2f}".format(self.gyro['x'])
            self.sprites['gyro_y'].text = "Y: {:3.2f}".format(self.gyro['y'])
            self.sprites['gyro_z'].text = "Z: {:3.2f}".format(self.gyro['z'])

            # Title Animation
            if(self.sprites['title_text'].x < -100):
                self.sprites['title_text'].x = 200
            else:
                self.sprites['title_text'].x -= 2

            # Update sprite, spritecounter and display    
            if(self.time + sleep_time) < time.monotonic():
                self.sprites['coin_sprite'][0] = counter
                self.update()
                counter += 1
                self.time = time.monotonic()
                if counter >= 14:
                    counter = 0           

        except Exception as e:
            # Fill the background with black
            self.fill(self.color('black'))
            # Draw the title bar
            self.draw_rectangle("title",0,0,240,40,self.color('red'))
            self.draw_text("title_text", 60, 25, "WS RP2040 1.28 Demo", self.color('white'), terminalio.FONT)
            # Draw the status bar
            self.draw_rectangle("battery",0,40,240,30,self.color('blue'))
            self.draw_text("charge_text", 20, 55, "Chg: {}".format(self.battery_status), self.color('white'), terminalio.FONT)
            self.draw_text("volt_text", 90, 55, "Vol: {:.2f}v".format(self.battery_voltage), self.color('white'), terminalio.FONT)
            self.draw_text("rev_text", 160, 55, "ARev: {}".format(self.qmi8658rev), self.color('white'), terminalio.FONT)
            # Draw the accelerometer data
            self.draw_rectangle("accelerometer",0,70,120,60,self.color('green'))
            self.draw_text("accelerometer_text", 20, 85, "Accel", self.color('black'), terminalio.FONT)
            self.draw_text("accel_x", 20, 95, "X: 0.0", self.color('black'), terminalio.FONT)
            self.draw_text("accel_y", 20, 105, "Y: 0.0", self.color('black'), terminalio.FONT)
            self.draw_text("accel_z", 20, 115, "Z: 0.0", self.color('black'), terminalio.FONT)
            # Draw the gyroscope data
            self.draw_rectangle("gyroscope",120,70,120,60,self.color('orange'))
            self.draw_text("gyroscope_text", 140, 85, "Gyro", self.color('black'), terminalio.FONT)
            self.draw_text("gyro_x", 140, 95, "X: 0.0", self.color('black'), terminalio.FONT)
            self.draw_text("gyro_y", 140, 105, "Y: 0.0", self.color('black'), terminalio.FONT)
            self.draw_text("gyro_z", 140, 115, "Z: 0.0", self.color('black'), terminalio.FONT)
            # Draw a bitmap sprite sheet and animate it
            self.draw_sprite("coin_sprite", 88, 150, 'bmp/sprite_coin.bmp', 14, 1, 0)
        return counter

hardware = wsRP2040128()
counter = 0
while True:
    counter = hardware.demo(counter)