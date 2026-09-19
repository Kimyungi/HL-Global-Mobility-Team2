# Idempotent MGM search/execution handshake, separate from legacy manual commands.
uint8 PREPARE=1
uint8 ACTIVATE=2
uint8 CANCEL=3
uint64 request_id
uint8 action
uint8 mission_mode
