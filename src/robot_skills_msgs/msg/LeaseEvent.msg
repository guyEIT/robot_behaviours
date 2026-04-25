# Structured event for lease lifecycle changes.
# Published on /skill_server/lease_events; BtExecutor listens for 'revoked'.

builtin_interfaces/Time stamp
string lease_id
string resource_id
string holder_id
string event                    # "acquired" | "renewed" | "released" | "revoked"
string reason                   # free-form; "ttl_expired" for TTL-driven revokes
