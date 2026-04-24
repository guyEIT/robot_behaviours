# One plate registered with the Liconic action server. Either sitting
# on the loading tray (cassette=0, position=0) or stored at a carousel
# site (cassette>=1, position>=1). `placed_at` is an ISO-8601 timestamp
# recorded when the registry entry was created; the server does not
# update it on subsequent reads.
string plate_name
string barcode
uint8 cassette
uint8 position
string placed_at
