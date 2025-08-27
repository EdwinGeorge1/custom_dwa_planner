from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header

def publish_markers(pub, trajectories, best_traj, clock):
    marker_array = MarkerArray()
    header = Header()
    header.frame_id = "odom"
    header.stamp = clock.now().to_msg()

    # Candidate trajectories (white, semi-transparent)
    for i, traj in enumerate(trajectories):
        marker = Marker()
        marker.header = header
        marker.ns = "dwa_candidates"
        marker.id = i
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.color.r = marker.color.g = marker.color.b = 1.0
        marker.color.a = 0.25
        marker.points = [Point(x=float(px), y=float(py), z=0.0) for (px, py) in traj]
        marker_array.markers.append(marker)

    # Best trajectory (green, bold)
    if best_traj:
        best_marker = Marker()
        best_marker.header = header
        best_marker.ns = "dwa_best"
        best_marker.id = 999
        best_marker.type = Marker.LINE_STRIP
        best_marker.action = Marker.ADD
        best_marker.scale.x = 0.04
        best_marker.color.g = 1.0
        best_marker.color.a = 1.0
        best_marker.points = [Point(x=float(px), y=float(py), z=0.0) for (px, py) in best_traj]
        marker_array.markers.append(best_marker)

    if marker_array.markers:
        pub.publish(marker_array)
