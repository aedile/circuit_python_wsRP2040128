# This is an implementation of the Waveshare 1.28" LCD RP2040 Board
# https://www.waveshare.com/wiki/1.28inch_LCD_RP2040_Board
# It uses the CircuitPython libraries and the GC9A01 display driver
# https://circuitpython.readthedocs.io/projects/gc9a01/en/latest/
# I had to implement my own battery and IMU classes because the
# existing ones didn't work with CircuitPython.  
# @author: Jesse R. Castro
# @TODO: Add menu class
# @TODO: Record accelerometer data to a file for calibration

import random
import time
from math import floor

import board
import busio
import displayio
import gc9a01
import terminalio
import analogio
import vectorio

from adafruit_display_text import label
from adafruit_bitmap_font import bitmap_font

# A generic class to describe battery status, should 
# work with any battery that has a voltage between 3.2V
# and 4.3V
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

        self.groups = {
            'default': displayio.Group()
        }

    # Add a new group to the display
    # groupname: the name of the group to add
    # returns: nothing
    def add_group(self, groupname):
        self.groups[groupname] = displayio.Group()

    # Draw a polygon on the display
    # points: a list of points, each point is a Tuple of 2 integers
    # x: the x coordinate of the 0,0 origin of the polygon
    # y: the y coordinate of the 0,0 origin of the polygon
    # color: a list of colors for the palette
    # groupname: the name of the group to add the polygon to
    # returns: the vectorio.Polygon object that was created
    def draw_polygon(self,points,x,y,color,groupname='default'):
        palette = displayio.Palette(len(color))
        for i in range(len(color)):
            palette[i] = color[i]
        polygon = vectorio.Polygon(pixel_shader=palette, points=points, x=x, y=y)
        self.groups[groupname].append(polygon)
        if self.auto_show:
            self.display.show(self.groups[groupname])
        return polygon

    # Draw a rectangle on the display
    # x: the x coordinate of the upper left corner of the rectangle
    # y: the y coordinate of the upper left corner of the rectangle
    # w: the width of the rectangle
    # h: the height of the rectangle
    # color: a list of colors for the palette
    # groupname: the name of the group to add the rectangle to
    # returns: the vectorio.Rectangle object that was created
    def draw_rectangle(self,x,y,w,h,color,groupname='default'):
        palette = displayio.Palette(len(color))
        for i in range(len(color)):
            palette[i] = color[i]
        rectangle = vectorio.Rectangle(pixel_shader=palette, width=w, height=h, x=x, y=y)
        self.groups[groupname].append(rectangle)
        if self.auto_show:
            self.display.show(self.groups[groupname])
        return rectangle
    
    # Draw a circle on the display
    # x: the x coordinate of the center of the circle
    # y: the y coordinate of the center of the circle
    # r: the radius of the circle
    # color: a list of colors for the palette
    # groupname: the name of the group to add the circle to
    # returns: the vectorio.Circle object that was created
    def draw_circle(self,x,y,r,color,groupname='default'):
        palette = displayio.Palette(len(color))
        for i in range(len(color)):
            palette[i] = color[i]
        circle = vectorio.Circle(pixel_shader=palette, radius=r, x=x, y=y)
        self.groups[groupname].append(circle)
        if self.auto_show:
            self.display.show(self.groups[groupname])
        return circle
    
    # Draw text on the display
    # x: the x coordinate of the upper left corner of the text
    # y: the y coordinate of the upper left corner of the text
    # text: the text to display
    # color: a list of colors for the pallette
    # font: the font to use
    # groupname: the name of the group to add the text to
    # returns: the label.Label object that was created
    def draw_text(self,x,y,text,color,font,groupname='default'):
        palette = displayio.Palette(len(color))
        for i in range(len(color)):
            palette[i] = color[i]
        text_area = label.Label(font, text=text, color=palette[0])
        text_area.x = x
        text_area.y = y
        self.groups[groupname].append(text_area)
        if self.auto_show:
            self.display.show(self.groups[groupname])
        return text_area    

    # Draw a bitmap file on the display
    # NOTE: proceed with caution as memory is low on this device
    # x: the x coordinate of the upper left corner of the bitmap
    # y: the y coordinate of the upper left corner of the bitmap
    # bitmap_path: the path to the bitmap file on the device
    # transparent_color: the color to make transparent, default is None
    # groupname: the name of the group to add the bitmap to
    # returns: the displayio.TileGrid object that was created
    def draw_bitmap(self,x,y,bitmap_path,transparent_color=None,groupname='default'):
        return self.draw_sprite(x,y,bitmap_path,1,1,0,transparent_color,groupname)
    
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
    # transparent_color: the color to make transparent
    # groupname: the name of the group to add the sprite to
    # returns: the displayio.TileGrid object that was created
    def draw_sprite(self,x,y,sprite_path,sprite_tiles_x,sprite_tiles_y,sprite_starting_tile,transparent_color = None, groupname='default'):
        bitmap = displayio.OnDiskBitmap(sprite_path)
        if transparent_color != None:
            bitmap.pixel_shader.make_transparent(transparent_color)
        sprite_tile_width = bitmap.width // sprite_tiles_x
        sprite_tile_height = bitmap.height // sprite_tiles_y
        tile_grid = displayio.TileGrid(bitmap, pixel_shader=bitmap.pixel_shader, width=1, height=1, tile_width=sprite_tile_width, tile_height=sprite_tile_height, default_tile=sprite_starting_tile, x=x, y=y)
        self.groups[groupname].append(tile_grid)
        if self.auto_show:
            self.display.show(self.groups[groupname])
        return tile_grid

    # Fill the display with a color
    # color: a list of colors for the palette
    # groupname: the name of the group to add the rectangle to
    # returns: the displayio.TileGrid object that was created
    def fill(self,color,groupname='default'):
        return self.draw_rectangle(0,0,self.width,self.height,color,groupname)

    # Show the display
    # returns: nothing
    def show(self):
        for group in self.groups:
            if not self.groups[group].hidden:
                self.display.show(self.groups[group])
            self.display.show(self.groups[group])
            break
    
# CircuitPython class representing the Waveshare RP2040 1.28" 
# development board.  This class is used to initialize the
# display and the pins used to communicate with the display as 
# well as the pins used to communicate with the accelerometer.
# It also has helper functions to make it easier to draw to the
# display.
class wsRP2040128(object):
    # Initialize the board
    # returns: nothing
    def __init__(self,initAccel=True, initBattery=True, initDisplay=True):
        # What we're actually gonna use
        self._use_display = initDisplay
        self._use_accel = initAccel
        self._use_battery = initBattery

        # Important - release the display from dev stuff
        if(self._use_display):
            displayio.release_displays()
        
        # Initialize hardware
        if(self._use_display):
            self._display = GC9A01_Display(True)
            # This is where we'll track our sprites
            self.sprites = {}
        
        if(self._use_accel):
            self._qmi8658 =QMI8658_Accelerometer()
            # The accelerometer revision
            self.qmi8658rev = self._qmi8658.rev
            # Accelerometer data
            self.accel = {
                'x': 0,
                'y': 0,
                'z': 0,
                
            }
            # This is a multiplier based on how long 
            # the device has been in motion so you can
            # ramp motion up instead of using linear.
            self.momentum = {
                'x': 0,
                'y': 0,
                'z': 0,
                'max': 10
            }
            # Gyroscope data
            self.gyro = {
                'x': 0,
                'y': 0,
                'z': 0
            }
            # These are current tilt values.  Possible
            # values are 'none', 'up' & 'down' for 'y', 
            # 'none', 'left' & 'right' for 'twist'
            # and 'none', 'left' & 'right' for 'x'
            self.tilt = {
                'x': 'none',
                'y': 'none',
                'twist': 'none'
            }  
            
            # Value of the current active tilt state
            # as well as when it was recorded. Possible values are
            # 'tilt up', 'tilt down', 'twist left', 'twist right'
            # and 'resting' for the state.  The time is the time
            # in seconds since the device was powered on.
            self.tilt_state = {'state': 'resting', 'time': time.monotonic()}
            # This is a list of the previous three tilt states
            # and when they occurred.  This is used to determine
            # if the user is shaking the device.
            self.tilt_history = [
                {'state': 'resting', 'time': time.monotonic()},
                {'state': 'resting', 'time': time.monotonic()},
                {'state': 'resting', 'time': time.monotonic()}
            ]
            self.cur_tilt_command = {'command': 'none', 'time': time.monotonic()}
            self.tilt_command_history = [
                {'command': 'none', 'time': time.monotonic()},
                {'command': 'none', 'time': time.monotonic()},
                {'command': 'none', 'time': time.monotonic()}
            ]
            # And this is the flag for activating a "combination" aka LRL
            self.combination = ''

        if(self._use_battery):
            self._battery = Battery()
            # Battery data
            self.battery_voltage = 0.0
            self.battery_percent = 0.0
            self.battery_charging = False
            self.battery_status = 'init'

        # Time tracker
        self.time = time.monotonic()

    # Passthrough method to draw a polygon on the display
    # sprite_id: text identifier for the sprite
    # points: a list of points that make up the polygon
    # x: the x coordinate of the 0,0 origin of the polygon
    # y: the y coordinate of the 0,0 origin of the polygon
    # color: the color of the polygon
    # group: the index of the displayio.Group object to add 
    #        the sprite to. Default is 'default'
    # returns: nothing
    def draw_polygon(self,sprite_id,points,x,y,color,group='default'):
        self.sprites[sprite_id] = self._display.draw_polygon(points,x,y,color,group)

    # Passthrough method to draw a circle on the display
    # sprite_id: text identifier for the sprite
    # x: the x coordinate of the center of the circle
    # y: the y coordinate of the center of the circle
    # r: the radius of the circle
    # color: the color of the circle
    # group: the index of the displayio.Group object to add
    #        the sprite to. Default is 'default'
    # returns: nothing
    def draw_circle(self,sprite_id,x,y,r,color,group='default'):
        self.sprites[sprite_id] = self._display.draw_circle(x,y,r,color,group)

    # Passthrough method to draw a rectangle on the display
    # sprite_id: text identifier for the sprite
    # x: the x coordinate of the upper left corner of the rectangle
    # y: the y coordinate of the upper left corner of the rectangle
    # w: the width of the rectangle
    # h: the height of the rectangle
    # color: the color of the rectangle
    # group: the index of the displayio.Group object to add
    #        the sprite to. Default is 'default'
    # returns: nothing    
    def draw_rectangle(self,sprite_id,x,y,w,h,color,group='default'):
        self.sprites[sprite_id] = self._display.draw_rectangle(x,y,w,h,color,group)

    # Passthrough method to draw text on the display
    # sprite_id: text identifier for the sprite
    # x: the x coordinate of the upper left corner of the text
    # y: the y coordinate of the upper left corner of the text
    # text: the text to display
    # color: the color of the text
    # font: the font to use
    # group: the index of the displayio.Group object to add
    #        the sprite to. Default is 'default'
    # returns: nothing
    def draw_text(self,sprite_id, x,y,text,color,font,group='default'):
        self.sprites[sprite_id] = self._display.draw_text(x,y,text,color,font,group)
    
    # Passthrough method to draw a bitmap file on the display
    # sprite_id: text identifier for the sprite
    # x: the x coordinate of the upper left corner of the bitmap
    # y: the y coordinate of the upper left corner of the bitmap
    # bitmap_path: the path to the bitmap file on the device
    # trans_color: the color to use for transparency
    # group: the index of the displayio.Group object to add
    #        the sprite to. Default is 'default'
    # returns: nothing
    def draw_bitmap(self,sprite_id, x,y,bitmap_path,trans_color=None,group='default'):
        self.sprites[sprite_id] = self._display.draw_bitmap(x,y,bitmap_path,trans_color,group)

    # Passthrough method to draw an animated sprite on the display
    # sprite_id: text identifier for the sprite
    # x: the x coordinate of the upper left corner of the sprite
    # y: the y coordinate of the upper left corner of the sprite
    # sprite_path: the path to the sprite file on the device
    # sprite_tiles_x: the number of tiles in the sprite in the x direction
    # sprite_tiles_y: the number of tiles in the sprite in the y direction
    # sprite_starting_tile: the starting tile in the sprite
    # trans_color: the color to use for transparency
    # group: the index of the displayio.Group object to add
    #        the sprite to. Default is 'default'
    # returns: nothing
    def draw_sprite(self,sprite_id,x,y,sprite_path,sprite_tiles_x,sprite_tiles_y,sprite_starting_tile,trans_color=None,group='default'):
        self.sprites[sprite_id] = self._display.draw_sprite(x,y,sprite_path,sprite_tiles_x,sprite_tiles_y,sprite_starting_tile,trans_color,group)

    # Passthrough method to fill the display with a color
    # color: the color to fill the display with
    # returns: nothing
    def fill(self,color,group='default'):
        self.sprites['background'] = self._display.fill(color,group)

    # Helper method to convert a color string to a color value
    # colstr: the color string
    # returns: the color value
    def color(self,colstr):
        colors = {
            'black': 0x000000,
            'white': 0xFFFFFF,
            'gray': 0x808080,
            'lightgray': 0xC0C0C0,
            'darkgray': 0x404040,
            'red': 0xFF0000,
            'green': 0x00FF00,
            'blue': 0x0000FF,
            'yellow': 0xFFFF00,
            'cyan': 0x00FFFF,
            'magenta': 0xFF00FF,
            'orange': 0xFFA500,
            'purple': 0x800080,
            'brown': 0xA52A2A,
            'pink': 0xFFC0CB,
            'lime': 0x00FF00,
            'teal': 0x008080,
            'maroon': 0x800000,
            'navy': 0x000080,
            'olive': 0x808000,
            'violet': 0xEE82EE,
            'turquoise': 0x40E0D0,
            'silver': 0xC0C0C0,
            'gold': 0xFFD700,
            'indigo': 0x4B0082,
            'coral': 0xFF7F50,
            'salmon': 0xFA8072,
            'tan': 0xD2B48C,
            'khaki': 0xF0E68C,
            'plum': 0xDDA0DD,
            'darkgreen': 0x006400
        }
        try:
            return [colors[colstr]]
        except:
            return [0x0000]

    # Helper method to get incremental steps
    # between two numbers.
    # start: the starting number
    # end: the ending number
    # steps: the number of steps to take
    # returns: a list of integers
    def _steps(self,start,end,steps):
        return [round(start + (end - start) * i / (steps - 1)) for i in range(steps)]        

    # Helper method to convert a binary color value to 
    # a list of three integers representing the red, green,
    # and blue values.
    # color: the binary color value
    # returns: a list of three integers
    def _color_to_rgb(self,color):
        return [color[0] >> 16 & 0xFF, color[0] >> 8 & 0xFF, color[0] & 0xFF]
    
    # Helper method to convert a list of three integers
    # representing the red, green, and blue values to a
    # binary color value.
    # rgb: a list of three integers
    # returns: the binary color value
    def _rgb_to_color(self,rgb):
        return [rgb[0] << 16 | rgb[1] << 8 | rgb[2]]

    # Helper method for color fades.  This method
    # accepts two binary color values and the number
    # of steps to take between the two colors.  It will
    # return a list of colors that can be used to fade.
    # start: the starting color
    # end: the ending color
    # returns: the color value
    def fade(self,start,end,steps):
        start_rgb = self._color_to_rgb(start)
        end_rgb = self._color_to_rgb(end)
        r_steps = self._steps(start_rgb[0],end_rgb[0],steps)
        g_steps = self._steps(start_rgb[1],end_rgb[1],steps)
        b_steps = self._steps(start_rgb[2],end_rgb[2],steps)
        return [self._rgb_to_color([r_steps[i],g_steps[i],b_steps[i]]) for i in range(steps)]
    
    # Show the display
    # returns: nothing
    def _show(self):
        self._display.show()  
    
    # Update the accelerometer data
    # returns: nothing
    def _update_accelerometer(self):
        xyz = self._qmi8658.read_xyz()
        accel = {}
        gyro = {}
        
        accel['x'] = xyz[1]
        accel['y'] = xyz[0]
        accel['z'] = xyz[2]
        gyro['x'] = xyz[3]
        gyro['y'] = xyz[4]
        gyro['z'] = xyz[5]       

        # Note: my device seems to have some calibration
        #       issues.  The values are not zeroed out
        #       when the board is not moving.  I'm going
        #       to try to compensate for this by subtracting
        #       a calibration value.  Manual for now but 
        #       I'll try to make this automatic in the future.
        accel['x'] += 0.01
        accel['y'] += 0.04
        accel['z'] += 1.11
        gyro['x'] -= 5.58
        gyro['y'] += 45.55
        gyro['z'] -= 0.20

        # And now we'll convert everything to integers to make math 
        # easier.  
        accel['x'] = int(accel['x'] * 10)
        accel['y'] = int(accel['y'] * -10)
        accel['z'] = int(accel['z'] * 10)
        gyro['x'] = int(gyro['x'])
        gyro['y'] = int(gyro['y'])
        gyro['z'] = int(gyro['z'])

        # And now we'll update the momentum values
        self.momentum['x'] += accel['x']
        self.momentum['y'] += accel['y']
        self.momentum['z'] += accel['z']
        if(self.momentum['x'] > self.momentum['max']):
            self.momentum['x'] = self.momentum['max']
        if(self.momentum['x'] < self.momentum['max']*-1):
            self.momentum['x'] = self.momentum['max']*-1
        if(self.momentum['y'] > self.momentum['max']):
            self.momentum['y'] = self.momentum['max']
        if(self.momentum['y'] < self.momentum['max']*-1):
            self.momentum['y'] = self.momentum['max']*-1
        if(self.momentum['z'] > self.momentum['max']):
            self.momentum['z'] = self.momentum['max']
        if(self.momentum['z'] < self.momentum['max']*-1):
            self.momentum['z'] = self.momentum['max']*-1

        # Then we'll update tilt status
        if(gyro['x'] > 0):
            self.tilt['x'] = 'right'
        elif(gyro['x'] < 0):
            self.tilt['x'] = 'left'
        else:
            self.tilt['x'] = 'center'
        if(gyro['y'] > 0):
            self.tilt['y'] = 'up'
        elif(gyro['y'] < 0):
            self.tilt['y'] = 'down'
        else:
            self.tilt['y'] = 'center'
        if(gyro['z'] > 0):
            self.tilt['twist'] = 'left'
        elif(gyro['z'] < 0):
            self.tilt['twist'] = 'right'
        else:
            self.tilt['twist'] = 'center'
        
        # Now we need to update the tilt_state and
        # tilt_history values based on which axis is 
        # currently being tilted the strongest.
        new_tilt_state = ''
        if(abs(gyro['x']) > abs(gyro['y']) and abs(gyro['x']) > abs(gyro['z'])):
            tilt_state = 'tilt ' + self.tilt['x']
        elif(abs(gyro['y']) > abs(gyro['x']) and abs(gyro['y']) > abs(gyro['z'])):
            tilt_state = 'tilt ' + self.tilt['y']
        elif(abs(gyro['z']) > abs(gyro['x']) and abs(gyro['z']) > abs(gyro['y'])):
            tilt_state = 'twist ' + self.tilt['twist']
        else:
            tilt_state = 'resting'
        if(tilt_state != self.tilt_state):
            self.tilt_state = tilt_state
            self.tilt_history.append(tilt_state)
            if(len(self.tilt_history) > 3):
                self.tilt_history.pop(0)    
        
        # And finally we'll check for command status
        cur_tilt_command = {'command':'none'}
        # if the last 3 readings are the same, we'll assume
        # that the user is holding the board in that position
        if self.tilt_history[0] == 'tilt left' and self.tilt_history[1] == 'tilt right':
            cur_tilt_command = {'command':'tilt left', 'time':time.monotonic()}
        elif self.tilt_history[0] == 'tilt right' and self.tilt_history[1] == 'tilt left':
            cur_tilt_command = {'command':'tilt right', 'time':time.monotonic()}
        elif self.tilt_history[0] == 'tilt up' and self.tilt_history[1] == 'tilt down':
            cur_tilt_command = {'command':'tilt up', 'time':time.monotonic()}
        elif self.tilt_history[0] == 'tilt down' and self.tilt_history[1] == 'tilt up':
            cur_tilt_command = {'command':'tilt down', 'time':time.monotonic()}
        elif self.tilt_history[0] == 'twist left' and self.tilt_history[1] == 'twist right':
            cur_tilt_command = {'command':'twist left', 'time':time.monotonic()}
        elif self.tilt_history[0] == 'twist right' and self.tilt_history[1] == 'twist left':
            cur_tilt_command = {'command':'twist right', 'time':time.monotonic()}
        else:
            # self.cur_tilt_command = 'resting'
            pass
        
        if cur_tilt_command['command'] != 'none':
            if cur_tilt_command['command'] != self.cur_tilt_command['command']:
                self.cur_tilt_command = cur_tilt_command
                self.tilt_command_history.append(cur_tilt_command)
                if(len(self.tilt_command_history) > 3):
                    self.tilt_command_history.pop(0)    

        # Look for combinations that have occurred in the last 2 seconds
        if (
            self.tilt_command_history[0]['command'] == 'twist left' and
            self.tilt_command_history[1]['command'] == 'twist right' and
            self.tilt_command_history[2]['command'] == 'twist left' and
            time.monotonic() - self.tilt_command_history[0]['time'] < 2
        ):
            self.combination = 'LRL'

        if (
            self.tilt_command_history[0]['command'] == 'twist right' and
            self.tilt_command_history[1]['command'] == 'twist left' and
            self.tilt_command_history[2]['command'] == 'twist right' and
            time.monotonic() - self.tilt_command_history[0]['time'] < 2
        ):
            self.combination = 'RLR'

        if (
            self.tilt_command_history[0]['command'] == 'tilt up' and
            self.tilt_command_history[1]['command'] == 'tilt down' and
            self.tilt_command_history[2]['command'] == 'tilt up' and
            time.monotonic() - self.tilt_command_history[0]['time'] < 2
        ):
            self.combination = 'UDU'
        if (
            self.tilt_command_history[0]['command'] == 'tilt up' and
            self.tilt_command_history[1]['command'] == 'tilt down' and
            self.tilt_command_history[2]['command'] == 'tilt up' and
            time.monotonic() - self.tilt_command_history[0]['time'] < 2
        ):
            self.combination = 'DUD'

        if self.combination != '':
            print('combination: {}'.format(self.combination))
            self.tilt_command_history = [
                {'command':'none', 'time':time.monotonic()},
                {'command':'none', 'time':time.monotonic()},
                {'command':'none', 'time':time.monotonic()}
            ]
        
        self.accel = accel
        self.gyro = gyro

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
        if(self._use_accel):
            self._update_accelerometer()
        if(self._use_battery):
            self._update_battery()
        if(self._use_display):
            self._show()

    # Demo code - run in the main loop, works if
    # you turn off hardware still.
    # sleep_time: the time to sleep between updates
    # returns: nothing
    def demo(self, sleep_time=0.05):
        if(self._use_display):
            # Fill the background with black
            self.draw_rectangle("demobg",0,0,240,240,self.color('black'))
            # Draw the title bar
            self.draw_rectangle("title",0,0,240,40,self.color('red'))
            self.draw_text("title_text", 60, 25, "WS RP2040 1.28 Demo", self.color('white'), terminalio.FONT)
            # Draw the status bar
            if(self._use_battery):
                self.draw_rectangle("battery",0,40,240,30,self.color('blue'))
                self.draw_text("charge_text", 20, 55, "Chg: {}".format(self.battery_status), self.color('white'), terminalio.FONT)
                self.draw_text("volt_text", 90, 55, "Vol: {:.2f}v".format(self.battery_voltage), self.color('white'), terminalio.FONT)
            if(self._use_accel):
                self.draw_text("rev_text", 160, 55, "ARev: {}".format(self.qmi8658rev), self.color('white'), terminalio.FONT)
            # Draw all accel data
            if(self._use_accel):
                # Draw the accelerometer data
                self.draw_rectangle("accelerometer",0,70,60,60,self.color('green'))
                self.draw_text("accelerometer_text", 20, 85, "Accel", self.color('black'), terminalio.FONT)
                self.draw_text("accel_x", 20, 95, "X: 0", self.color('black'), terminalio.FONT)
                self.draw_text("accel_y", 20, 105, "Y: 0", self.color('black'), terminalio.FONT)
                self.draw_text("accel_z", 20, 115, "Z: 0", self.color('black'), terminalio.FONT)
                # Draw the gyroscope data
                self.draw_rectangle("gyroscope",60,70,60,60,self.color('orange'))
                self.draw_text("gyroscope_text", 80, 85, "Gyro", self.color('black'), terminalio.FONT)
                self.draw_text("gyro_x", 80, 95, "X: 0", self.color('black'), terminalio.FONT)
                self.draw_text("gyro_y", 80, 105, "Y: 0", self.color('black'), terminalio.FONT)
                self.draw_text("gyro_z", 80, 115, "Z: 0", self.color('black'), terminalio.FONT)
                # Draw the Tilt data
                self.draw_rectangle("momentum",120,70,60,60,self.color('magenta'))
                self.draw_text("momentum_text", 140, 85, "Tilt", self.color('black'), terminalio.FONT)
                self.draw_text("mom_x", 140, 95, "X: 0", self.color('black'), terminalio.FONT)
                self.draw_text("mom_y", 140, 105, "Y: 0", self.color('black'), terminalio.FONT)
                self.draw_text("mom_z", 140, 115, "Z: 0", self.color('black'), terminalio.FONT)
                # Draw current tilt status and command
                self.draw_rectangle("tilt_status",180,70,60,60,self.color('cyan'))
                self.draw_text("tilt_status_text", 200, 85, "Stat", self.color('black'), terminalio.FONT)
                self.draw_text("tilt_status", 200, 95, "resting", self.color('black'), terminalio.FONT)
                self.draw_text("tilt_command", 200, 105, "resting", self.color('black'), terminalio.FONT)
            # Draw a cicle
            self.draw_circle("circle", 40, 170, 20, self.color('yellow'))
            # Draw a polygon
            points = [
                (15,0),
                (11,10),
                (0,10),
                (8,20),
                (5,29),
                (15,21),
                (25,29),
                (22,20),
                (29,10),
                (19,10)
            ]
            self.draw_polygon("polygon", points, 160, 160, self.color('brown'))
        else:
            print("Display not intialized")

        while True:
            if(self._use_display):
                # Update the sensor info
                if(self._use_battery):
                    self.sprites['volt_text'].text = "Vol: {:.2f}v".format(self.battery_voltage)
                    self.sprites['charge_text'].text = "Chg: {}".format(self.battery_status)
                if(self._use_accel):    
                    self.sprites['accel_x'].text = "X: {}".format(self.accel['x'])
                    self.sprites['accel_y'].text = "Y: {}".format(self.accel['y'])
                    self.sprites['accel_z'].text = "Z: {}".format(self.accel['z'])
                    self.sprites['gyro_x'].text = "X: {}".format(self.gyro['x'])
                    self.sprites['gyro_y'].text = "Y: {}".format(self.gyro['y'])
                    self.sprites['gyro_z'].text = "Z: {}".format(self.gyro['z'])
                    self.sprites['mom_x'].text = "X: {}".format(self.momentum['x'])
                    self.sprites['mom_y'].text = "Y: {}".format(self.momentum['y'])
                    self.sprites['mom_z'].text = "Z: {}".format(self.momentum['z'])
                    self.sprites['tilt_status'].text = "{}".format(self.tilt_state)
                    self.sprites['tilt_command'].text = "{}".format(self.cur_tilt_command['command'])

                # Title Animation
                if(self.sprites['title_text'].x < -100):
                    self.sprites['title_text'].x = 200
                else:
                    self.sprites['title_text'].x -= 1

                # Update sprite, spritecounter and display    
                if(self.time + sleep_time) < time.monotonic():
                    self.update()
                    self.time = time.monotonic()

            # Break    
            if(self.combination == 'LRL'):
                self.combination = ''
                sprites = [
                    'demobg',
                    'title',
                    'title_text',
                    'circle',
                    'polygon'
                ]
                if(self._use_battery):
                    sprites.append('battery')
                    sprites.append('volt_text')
                    sprites.append('charge_text')
                if(self._use_accel):
                    sprites.append('accelerometer')
                    sprites.append('accelerometer_text')
                    sprites.append('rev_text')
                    sprites.append('accel_x')
                    sprites.append('accel_y')
                    sprites.append('accel_z')
                    sprites.append('gyroscope')
                    sprites.append('gyroscope_text')
                    sprites.append('gyro_x')
                    sprites.append('gyro_y')
                    sprites.append('gyro_z')
                    sprites.append('momentum')
                    sprites.append('momentum_text')
                    sprites.append('mom_x')
                    sprites.append('mom_y')
                    sprites.append('mom_z')
                    sprites.append('tilt_status')
                    sprites.append('tilt_status_text')
                    sprites.append('tilt_command')
                
                for sprite in sprites:
                    self._display.groups['default'].remove(self.sprites[sprite])
                break    
            pass
    
    # Demo using drawing and accelerator.  We're creating
    # a ball that reads the accelerometer and moves around
    # the screen.  When it falls off the table,
    # the game is over and the ball is reset to the center.
    # sleep_time: the time to sleep between updates
    # returns: nothing
    def ball_demo(self, sleep_time=0.05):
        # Initializations
        self.combination = ''
        self.draw_rectangle("ballbg", 0,0,240,240,self.color('blue'))
        self.draw_rectangle("table", 35,35,170,170,self.color('orange'))
        self.draw_circle("ball", 120, 120, 10, self.color('purple'))
        
        while True:
            # Main game loop
            # Update the sprites based on the accelerometer
            curposx = self.sprites['ball'].x
            curposy = self.sprites['ball'].y
            newposx = curposx + self.momentum['x']
            newposy = curposy + self.momentum['y']
            self.sprites['ball'].x = newposx
            self.sprites['ball'].y = newposy
            
            # Check for collision with the walls
            if (
                self.sprites['ball'].x < 22 
                or self.sprites['ball'].x > 215 
                or self.sprites['ball'].y < 22 
                or self.sprites['ball'].y > 215
                ):
                self.sprites['ball'].x = 120
                self.sprites['ball'].y = 120
                self.momentum['x'] = 0
                self.momentum['y'] = 0

            self.update() 

            if(self.combination == 'LRL'):
                self.combination = ''
                self._display.groups['default'].remove(self.sprites['ballbg'])
                self._display.groups['default'].remove(self.sprites['table'])
                self._display.groups['default'].remove(self.sprites['ball'])
                break
        
            time.sleep(sleep_time)

     # Demo using drawing and accelerator.  We're creating
    
    # This is the banner screen.  Very simple, just a banner
    # that scrolls across the screen.
    # banner_text: the text to display in the banner
    # sleep_time: the time to sleep between updates
    # returns: nothing
    def banner_demo(self, banner_text, sleep_time=0.05):
        self.combination = ''
        self.draw_rectangle("bannerbg", 0,0,240,240,self.color('black'))
        
        # We are going to draw the badge rim by making a series of concentric circles
        # starting with the outermost circle and working our way in.  We'll use the 
        # fade method to give the circles a 3d effect.
        cfarr = self.fade(self.color('orange'), self.color('white'), 9)
        colorfade = cfarr + cfarr[::-1] + [self.color('black')]
        for i in range(0, len(colorfade)):
            self.draw_circle("rim_{}".format(i), 120, 120, 120-i, colorfade[i])

        self.draw_text("banner_text", 0, 100, banner_text, self.color('magenta'), bitmap_font.load_font("font/mfbold.bdf"))
        
        while True:
            # Main game loop
            # Update the sprites based on the accelerometer
            curposx = self.sprites['banner_text'].x
            newposx = curposx - 1
            self.sprites['banner_text'].x = newposx
            # Check for collision with the walls
            if (self.sprites['banner_text'].x < (0 - self.sprites['banner_text'].width)):
                self.sprites['banner_text'].x = 240
            
            self.update() 

            if(self.combination == 'LRL'):
                self.combination = ''
                self._display.groups['default'].remove(self.sprites['bannerbg'])
                self._display.groups['default'].remove(self.sprites['banner_text'])
                for i in range(0, len(colorfade)):
                    self._display.groups['default'].remove(self.sprites['rim_{}'.format(i)])
                break
        
            time.sleep(sleep_time)

    # a ball that reads the accelerometer and moves around
    # the screen.  When it falls off the table,
    # the game is over and the ball is reset to the center.
    # sleep_time: the time to sleep between updates
    # returns: nothing
    def old_menu(self, sleep_time=0.05):
        # Initializations
        self.fill(self.color('black'))
        self.draw_text("prompt_text", 70, 100, "Tilt to choose:", self.color('white'), terminalio.FONT)
        self.draw_text("bottom_choice_text", 100, 220, "Banner", self.color('white'), terminalio.FONT)
        self.draw_text("top_choice_text", 90, 10, "Settings", self.color('white'), terminalio.FONT)
        self.draw_text("left_choice_text", 10, 100, "Games", self.color('white'), terminalio.FONT)
        self.draw_text("right_choice_text", 220, 100, "Off", self.color('white'), terminalio.FONT)
        self.draw_circle("cursor", 120, 120, 5, self.color('white'))
        
        while True:
            # Main game loop
            # Update the sprites based on the accelerometer
            curposx = self.sprites['cursor'].x
            curposy = self.sprites['cursor'].y
            newposx = curposx + self.momentum['x']
            newposy = curposy + self.momentum['y']
            self.sprites['cursor'].x = newposx
            self.sprites['cursor'].y = newposy
            
            # Reset ball position
            def reset_cursor():
                self.sprites['cursor'].x = 120
                self.sprites['cursor'].y = 120
                self.momentum['x'] = 0
                self.momentum['y'] = 0
            
            def remove_sprites():
                self._display.groups['default'].remove(self.sprites['prompt_text'])
                self._display.groups['default'].remove(self.sprites['bottom_choice_text'])
                self._display.groups['default'].remove(self.sprites['top_choice_text'])
                self._display.groups['default'].remove(self.sprites['left_choice_text'])
                self._display.groups['default'].remove(self.sprites['right_choice_text'])
                self._display.groups['default'].remove(self.sprites['cursor'])
                
            def add_sprites():
                reset_cursor()
                self._display.groups['default'].append(self.sprites['prompt_text'])
                self._display.groups['default'].append(self.sprites['bottom_choice_text'])
                self._display.groups['default'].append(self.sprites['top_choice_text'])
                self._display.groups['default'].append(self.sprites['left_choice_text'])
                self._display.groups['default'].append(self.sprites['right_choice_text'])
                self._display.groups['default'].append(self.sprites['cursor'])

            # Check for collision with the walls
            if (self.sprites['cursor'].x < 22):
                # left option
                remove_sprites()
                self.ball_demo()
                add_sprites()
            elif (self.sprites['cursor'].x > 215):
                # right option
                remove_sprites()
                self.ball_demo()
                add_sprites()
            elif(self.sprites['cursor'].y < 22):
                remove_sprites()
                self.demo()
                add_sprites()  
            elif(self.sprites['cursor'].y > 215):
                remove_sprites()
                self.banner_demo('Easily Amused')
                add_sprites()
            self.update()        
            time.sleep(sleep_time)


    # New menu.  User is presented with options to choose from.  
    # User can select which option to choose by tilting the board
    # backwards and forwards.  If the user does not make a selection
    # within 20 seconds, the menu will automatically select the banner
    # option.
    # sleep_time: the time to sleep between updates
    # returns: nothing
    def main_menu(self, sleep_time=0.05):
        self.fill(self.color('black'))
        self.draw_text("prompt_text", 40, 80, "Please choose an option:", self.color('white'), terminalio.FONT)
        choices = ['Banner', 'Settings', 'Games', 'Off']
        selection = 0
        for i in range(0, len(choices)):
            self.draw_text("choice_{}".format(i), 100, 100 + (i * 20), choices[i], self.color('white'), terminalio.FONT)

        self.draw_text("selector", 90, 100, "*", self.color('white'), terminalio.FONT)

        def select(selection, choices):
            if selection >= len(choices):
                    selection = 0
            elif selection < 0:
                    selection = len(choices) - 1

            self.sprites['selector'].y = 100 + (selection * 20)
            return selection

        while True:
            com = self.combination
            if(com != ''):
                self.combination = ''
            if(com == 'UDU'):
               selection -= 1
               selection = select(selection, choices)     
            elif(com == 'DUD'):
               selection += 1
               selection = select(selection, choices)
            elif(com == 'RLR'):
                if selection == 0:
                    self.banner_demo('Easily Amused')
                elif selection == 1:
                    self.demo()
                elif selection == 2:
                    self.ball_demo()
                elif selection == 3:
                    self.demo()
            self.update()                    
            pass
        



hardware = wsRP2040128()
hardware.main_menu()