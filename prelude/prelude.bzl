# Minimal prelude: re-export configuration built-ins so every BUCK file can use
# platform(), constraint_setting(), and constraint_value() without a load().
platform          = native.platform
constraint_setting = native.constraint_setting
constraint_value  = native.constraint_value
