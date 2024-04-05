import rospy
import math
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import TwistStamped
import GPS

class RosNMEADriver(object):
    def __init__(self, node_name):
        self.fix_pub = rospy.Publisher('fix', NavSatFix, queue_size=1)
        self.vel_pub = rospy.Publisher('vel', TwistStamped, queue_size=1)
        self.time_ref_pub = rospy.Publisher('time', TimeReference, queue_size=1)
        
        self.time_ref_source = rospy.get_param('time_ref-source', None)
        self.use_RMC = rospy.get_param('useRMC', False)
        self.covariance_matrix = rospy.get_param('covariance_matrix', None)
        
    def add_sentence(self, nmea_string, frame_id, timestamp=None):
        if timestamp:
            current_time = timestamp
        else:
            current_time = rospy.get_rostime()
            current_fix = NavSatFix()
            current_fix.header.stamp = current_time
            current_fix.header.frame_id = frame_id
            current_time_ref = TimeReference()
            current_time_ref.header.stamp = current_time
            current_time_ref.header.frame_id = frame_id
            if self.time_ref_source:
                current_time_ref.source = self.time_ref_source
            else:
                current_time_ref.source = frame_id
                
        if not self.use_RMC and '$GNGGA' in GPS:
            data = GPS['$GNGGA']
            gps_qual = data['fix_type']
            if gps_qual == 0:
                current_fix.status.status = NavSatStatus.STATUS_NO_FIX
            elif gps_qual == 1:
                current_fix.status.status = NavSatStatus.STATUS_FIX
            elif gps_qual == 2:
                current_fix.status.status = NavSatStatus.STATUS_SBAS_FIX
            elif gps_qual in (4, 5):
                current_fix.status.status = NavSatStatus.STATUS_GBAS_FIX
            else:
                current_fix.status.status = NavSatStatus.STATUS_NO_FIX
            
            current_fix.status.status = NavSatStatus.STATUS_GPS
            
            current_fix.header.stamp = current_time
            
            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            current_fix.latitude = latitude

            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude
            current_fix.longitude = longitude
            
            if self.covariance_matrix and isinstance(self.covariance_matrix, list) and len(self.covariance_matrix) == 9:
                for i in range(9):
                    current_fix.position_covariance[i] = self.covariance_matrix[i]
                current_fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_KNOWN
            else:
                hdop = data['hdop']
                current_fix.position_covariance[0] = hdop ** 2
                current_fix.position_covariance[4] = hdop ** 2
                current_fix.position_covariance[8] = (2 * hdop) ** 2  # FIXME
                current_fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            
            # Altitude is above ellipsoid, so adjust for mean-sea-level
            altitude = data['altitude'] + data['mean_sea_level']
            current_fix.altitude = altitude

            self.fix_pub.publish(current_fix)
            
            if not math.isnan(data['utc_time']):
                current_time_ref.time_ref = rospy.Time.from_sec(data['utc_time'])
                self.time_ref_pub.publish(current_time_ref)
            
            return current_fix
        
        elif '$GNGGA' in GPS:
            data = GPS['$GNGGA']
            
            if self.use_RMC:
                if data['fix_valid']:
                    current_fix.status.status = NavSatStatus.STATUS_FIX
                else:
                    current_fix.status.status = NavSatStatus.STATUS_NO_FIX

                current_fix.status.service = NavSatStatus.SERVICE_GPS

                latitude = data['latitude']
                if data['latitude_direction'] == 'S':
                    latitude = -latitude
                current_fix.latitude = latitude

                longitude = data['longitude']
                if data['longitude_direction'] == 'W':
                    longitude = -longitude
                current_fix.longitude = longitude

                current_fix.altitude = float('NaN')
                current_fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_UNKNOWN
                    
                self.fix_pub.publish(current_fix)
                
                if not math.isnan(data['utc_time']):
                    current_time_ref.time_ref = rospy.Time.from_sec(data['utc_time'])
                    self.time_ref_pub.publish(current_time_ref)
                    
            if data['fix_valid']:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * \
                    math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * \
                    math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)
                
            return current_fix
        else:
            return None
    
    def get_frame_id():
        frame_id = rospy.get_param('~frame_id', 'gps')
        if frame_id[0] != "/":
            """Add the TF prefix"""
            prefix = ""
            prefix_param = rospy.search_param('tf_prefix')
            if prefix_param:
                prefix = rospy.get_param(prefix_param)
                if prefix[0] != "/":
                    prefix = "/%s" % prefix
            return "%s/%s" % (prefix, frame_id)
        else:
            return frame_id