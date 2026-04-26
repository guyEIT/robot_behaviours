# Snapshot of a single lease held by the LeaseBroker.
# Published via /skill_server/leases as part of a JSON list.

string lease_id
string resource_id
string holder_id
builtin_interfaces/Time acquired_at
builtin_interfaces/Time expires_at
float32 ttl_sec
