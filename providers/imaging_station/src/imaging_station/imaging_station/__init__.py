"""imaging_station — generic plate imaging provider.

Implements the robot_skills_msgs/action/ImagePlate action server. Ships a
sim backend (writes placeholder files after a configurable delay) and
will gain a real driver backend once imaging hardware is selected.
"""
