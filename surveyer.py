import logging
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.log import LogConfig
from threading import Timer

import math



URI = uri_helper.uri_from_env(default = 'radio://0/80/2M/E7E7E7E7E7')

if len(sys.argv) > 1:
    URI = sys.argv[1]

# Only output errors from the logging framework
logging.basicConfig(level = logging.ERROR)
global dic
class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Position', period_in_ms=100)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        # self._lg_stab.add_variable('stabilizer.roll', 'float')
        # self._lg_stab.add_variable('stabilizer.pitch', 'float')
        # self._lg_stab.add_variable('stabilizer.yaw', 'float')
        # The fetch-as argument can be set to FP16 to save space in the log packet
        # self._lg_stab.add_variable('pm.vbat', 'FP16')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        # Start a timer to disconnect in 10s
        t = Timer(1, self._cf.close_link)
        t.start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        print(f'[{timestamp}][{logconf.name}]: ', end='')
        global dic
        dic = {}
        for name, value in data.items():
            dic[name[-1]] = math.ceil(value * 100) / 100
            print(f'{name}: {value:3.3f} ', end='')
        print("MyDic:", dic)
        
    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

def simple_sequence():
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        with PositionHlCommander(scf, default_velocity=0.1, controller=PositionHlCommander.CONTROLLER_PID) as pc:


          #  pc.forward(1.0)
          #  pc.left(1.0)
          #  pc.back(1.0)
            print("Starting Position: "+str(pc.get_position()))
            pc.land()

def is_close(range):
    MIN_DISTANCE = 0.5  

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

def is_far(range):
    MAX_DISTANCE = 0.9  

    if range is None:
        return False
    else:
        return range > MAX_DISTANCE

def correction(multiranger, mc):
    if is_close(multiranger.front) != False:
                mc.back(0.5-multiranger.front)
    elif is_far(multiranger.front) != False:
                mc.forward(multiranger.front-0.5)

def box_path(mc, multiranger, lr, ud, v, r = 0.5,l = 0.5,d = 0.5,u = 0.5, step = 0.1):
    
    while r>=0.1 and l>=0.1 and u>=0.1 and d>=0.1: 

        if lr == 'l':
            mc.left(l, v)
            print('Going left')
            time.sleep(1)
            l-=0.1
            if ud == 'u':
                mc.up(u, v)
                print('Going up')
                u-=0.1
                time.sleep(1)
                mc.right(r, v)
                print('Going right')
                r-=0.1
                time.sleep(1)
                d-=0.1
                mc.down(d, v)
                print('Going down')
                time.sleep(1)
                
            elif ud == 'd':
                mc.down(d, v)
                print('Going down')
                d-=0.1
                time.sleep(1)
                mc.right(r, v)
                print('Going right')
                r-=0.1
                time.sleep(1)
                u-=0.1
                mc.up(u, v)
                print('Going up')
                time.sleep(1)
                
            else:
                print("Error use values u or d only")

        elif lr == 'r':
            correction(multiranger, mc)
            mc.right(r, v)
            correction(multiranger, mc)
           
            print('Going right')
            time.sleep(1)
            r-=0.1
            if ud == 'u':
                mc.up(u, v)
                print('Going up')
                u-=0.1
                time.sleep(1)
                mc.left(l, v)
                print('Going left')
                l-=0.1
                time.sleep(1)
                d-=0.1
                mc.down(d, v)
                print('Going down')
                time.sleep(1)
                
            elif ud == 'd':
                correction(multiranger, mc)
                mc.down(d, v)
                correction(multiranger, mc)
               

                print('Going down')
                d-=0.1
                time.sleep(1)
                correction(multiranger, mc)
                mc.left(l, v)
                correction(multiranger, mc)
                

                print('Going left')
                l-=0.1
                time.sleep(1)
                u-=0.1
                correction(multiranger, mc)
                mc.up(u, v)
                correction(multiranger, mc)

                print('Going up')
                time.sleep(1)
                
            else:
                print("Error use values u or d only")
        else:
            print("Error use values l or r only")

    print('Box Path Complete')

def fly_drone(lr, ud, height = 0.5, forward_step = 0.1, v = 0.2):
    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache = './cache')
    with SyncCrazyflie(URI, cf = cf) as scf:
        with MotionCommander(scf) as mc:
            with Multiranger(scf) as multiranger:
                with PositionHlCommander(
                scf,
               # x=0.0, y=0.0, z=0.0,
                default_velocity=v,
                default_height=height,
                controller=PositionHlCommander.CONTROLLER_PID) as pc:
                    dist_traveled = 0
                    
                    mc.up(height)
                    while is_close(multiranger.front) == False:
                        mc.forward(forward_step, v)
                        dist_traveled+=forward_step
                        print('Going forward')
                    time.sleep(0.5)
                    mc.back(0.1)
                    mc.forward(0.01)
                    time.sleep(1)
                    box_path(mc, multiranger, lr,ud, v,r = 0.7,l = 0.7, d = 0.4,u = 0.4, step = 0.1)
                    print('Now Going Back')
                    #pc.go_to(0.0, 0.0, 0.0)
                    mc.back(dist_traveled-1.0)
                    mc.forward(0.15)
                    time.sleep(1)
                    print('Landing')
                    mc.stop()
                    time.sleep(1)

                print('Survey Complete')


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    le = LoggingExample(URI)
    while le.is_connected:
        time.sleep(1)
    print('Initial Position:', dic)
    lr = str(input('Box to the left or right? : ')).lower() #Only accepts l or r
    ud = str(input('Box up or down? : ')).lower() #Only accepts u or d
    h = float(input('Initial height? : '))
    fly_drone(lr, ud, h) #Custom forward steap and velocity can also be passed here
