# A registered off-deck handoff site (incubator dock, cytomat entry,
# centrifuge port, etc.) the iSWAP can reach. One record serves both
# the manual-jog HandoffTransfer recipe (uses x,y,z,plate_width,rotation,
# wrist) and the firmware-atomic HotelTransfer / MoveResource(to_handoff)
# path (uses x,y,z,plate_width,grip_direction,hotel_depth,
# hotel_clearance_height).
string name
float32 x
float32 y
float32 z
float32 plate_width                    # iSWAP grip width [mm]
string rotation                        # shoulder rotation for HandoffTransfer (FRONT/LEFT/RIGHT/BACK)
string wrist                           # wrist orientation for HandoffTransfer (e.g. STRAIGHT)
string grip_direction                  # GripDirection for HotelTransfer/MoveResource (FRONT/BACK/LEFT/RIGHT)
float32 hotel_depth                    # mm of firmware-planned approach depth
float32 hotel_clearance_height         # mm vertical clearance into the hotel
