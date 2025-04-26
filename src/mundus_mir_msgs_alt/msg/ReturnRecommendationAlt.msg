# Timestamp for when this recommendation was calculated
builtin_interfaces/Time stamp

# Primary recommendation flag
bool should_return

# Current state
float64 current_battery_level      # Current battery percentage (0-100)
float64 distance_to_dock           # Current distance to dock in meters
float64 current_speed              # Current ROV speed in m/s
float64 current_consumption_rate   # Current battery consumption rate (%/second)

# Return journey estimates
float64 estimated_return_energy    # Estimated battery percentage needed for return
float64 estimated_time_to_return   # Estimated time to return in seconds
float64 minimum_battery_needed     # Minimum battery percentage needed (including safety margin)

# Safety margins
float64 safety_margin_percent      # Current safety margin as a percentage
float64 battery_safety_threshold   # Minimum safe battery level to maintain

# Additional info
float64[] consumption_rates        # Array of recent consumption rates for analysis
float64[] speeds                   # Array of recent speeds
float64[] timestamps               # Array of timestamps for the consumption data