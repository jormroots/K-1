import pyproj
import numpy as np
from new_GPS import lat, lot, alt

# Define a reference point (latitude, longitude, altitude) in GPS coordinates
ref_lat, ref_lon, ref_alt = lat, lot, alt  # Example: San Francisco, CA

# Initialize a projection object for WGS84 (GPS)
wgs84 = pyproj.Proj(proj='latlong', datum='WGS84')

# Convert reference point to ECEF coordinates
ref_ecef_x, ref_ecef_y, ref_ecef_z = pyproj.transform(wgs84, pyproj.Proj(proj='geocent'), ref_lon, ref_lat, ref_alt)

# Define a point to convert (latitude, longitude, altitude) in GPS coordinates
point_lat, point_lon, point_alt = 37.7749, -122.4194, 100  # Example: San Francisco, CA

# Convert point to ECEF coordinates
point_ecef_x, point_ecef_y, point_ecef_z = pyproj.transform(wgs84, pyproj.Proj(proj='geocent'), point_lon, point_lat, point_alt)

# Calculate the local tangent plane (LTP) conversion matrix
# First, calculate the geodetic to ENU transformation matrix
# See https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ECEF_to_ENU
lon_rad = np.deg2rad(ref_lon)
lat_rad = np.deg2rad(ref_lat)

# Transformation matrix elements
t11 = -np.sin(lon_rad)
t12 = np.cos(lon_rad)
t13 = 0
t21 = -np.sin(lat_rad) * np.cos(lon_rad)
t22 = -np.sin(lat_rad) * np.sin(lon_rad)
t23 = np.cos(lat_rad)
t31 = np.cos(lat_rad) * np.cos(lon_rad)
t32 = np.cos(lat_rad) * np.sin(lon_rad)
t33 = np.sin(lat_rad)

# Local tangent plane (LTP) conversion matrix
rotation_matrix = np.array([[t11, t12, t13],
                            [t21, t22, t23],
                            [t31, t32, t33]])

# Convert ECEF coordinates to ENU coordinates
delta_east, delta_north, delta_up = np.dot(rotation_matrix, np.array([point_ecef_x - ref_ecef_x, point_ecef_y - ref_ecef_y, point_ecef_z - ref_ecef_z]))

print("East:", delta_east)
print("North:", delta_north)
print("Up:", delta_up)
