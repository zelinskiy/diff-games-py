from pymavlink import mavutil, mavwp

import time
import signal
import sys

# python2 sim_vehicle.py -v ArduPlane --console --map -w

class Drone:
    def __init__(self, connString, baudRate=115200):
        print("Initializing drone")
        self.master = mavutil.mavlink_connection(connString, baud=baudRate, autoreconnect=True)
        self.master.wait_heartbeat(blocking=True)
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            1, # Hz
            1) # Start
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            1, # Hz
            1) # Start
        print(mavutil.mavlink)
        print("Heartbeat received")

    def gpsIntToFloat(self, x):
        return float((str(x)[:-7]) + '.' + str(x)[-7:])

    def _arm(self, arm):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
            0, # confirmation
            arm, # arm
            0,
            0,
            0,
            0,
            0,
            0)
        msg = self.master.recv_match(
            type=['COMMAND_ACK'],
            condition='COMMAND_ACK.command=='+str(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM),
            blocking=True)
        if msg.result != 0:
            raise Exception("Unable to arm/disarm")


    def arm(self):
        print("Arming")
        self._arm(1)
        print("Armed")


    def disarm(self):
        print("Disarming")
        self._arm(0)
        print("Disarmed")


    def abort(self):
        print("Aborting")
        mavutil.mavfile_global.set_mode_rtl()


    def abort_exit(self, signal, frame):
        self.abort()
        sys.exit()


    def set_mode_loiter(self):
        mavutil.mavfile_global.set_mode_loiter()
        msg = self.master.recv_match(
            type=['COMMAND_ACK'],
            condition='COMMAND_ACK.command==' + str(mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM),
            blocking=True)
        if(msg.result != 0):
            raise Exception("Unable to switch mode")

        print("switched mode")

    def _sendMission(self, mission_items):
        wp = mavwp.MAVWPLoader()
        seq = 1
        for item in mission_items:
            item.seq = seq
            wp.add(item)
            seq += 1

        self.master.waypoint_clear_all_send()
        self.master.waypoint_count_send(wp.count())

        for i in range(wp.count()):
            msg = self.master.recv_match(type=['MISSION_REQUEST'], blocking=True)
            self.master.mav.send(wp.wp(msg.seq))
            print('Sending waypoint {0}'.format(msg.seq))

        print("Mission uploaded")

    def startMission(self):
        print("Starting mission")
        self.set_mode_loiter()
        self.arm()

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component, # target_component
            mavutil.mavlink.MAV_CMD_MISSION_START, # command
            0,
            1, # first WP
            mavwp.MAVWPLoader().count(), # last WP
            0,
            0,
            0,
            0,
            0)
        msg = self.master.recv_match(
            type=['COMMAND_ACK'],
            condition='COMMAND_ACK.command==' + str(mavutil.mavlink.MAV_CMD_MISSION_START),
            blocking=True)
        if(msg.result != 0):
            raise Exception("Unable to start mission")

        print("Mission started")


    def takeoff(self, altitude):
        takeoff_wp = mavutil.mavlink.MAVLink_mission_item_message(
            self.master.target_system,
            self.master.target_component,
            -1, # to be set
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0, # min pitch
            0,
            0,
            0,
            0,
            0,
            altitude)

        # first wp is ignored somewhy
        self._sendMission([takeoff_wp, takeoff_wp])
        self.startMission()

    def sendWaypoints(self, wps, alt=20):
        takeoff_wp = mavutil.mavlink.MAVLink_mission_item_message(
            self.master.target_system,
            self.master.target_component,
            -1, # to be set
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0, # min pitch
            0,
            0,
            0,
            0,
            0,
            alt)
        wps_ = [takeoff_wp, takeoff_wp]
        for wp in wps:
            wp_ = mavutil.mavlink.MAVLink_mission_item_message(
                self.master.target_system,
                self.master.target_component,
                -1, # to be set
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
                0,
                0,
                0, # min pitch
                0,
                0,
                0,
                wp[0],
                wp[1],
                alt)
            wps_.append(wp_)
        self._sendMission(wps_)


    def getPosition(self):
        print("Waiting drone position...")
        msg = self.master.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True)
        self.yaw = msg.hdg / 100.0
        self.lat = self.gpsIntToFloat(msg.lat)
        self.lon = self.gpsIntToFloat(msg.lon)
        self.alt = float(msg.relative_alt) / 1000.0
        self.pos = [self.lat, self.lon, self.alt]
        print("Drone position received")
        print(self.pos)
        return self.pos



if __name__ == "__main__":
    drone = Drone("tcp:127.0.0.1:5763")
    # signal.signal(signal.SIGINT, drone.abort_exit)
    drone.takeoff(15)
    while(True):
        time.sleep(1)
        drone.getPosition()
