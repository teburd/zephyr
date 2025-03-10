zephyr_library()
zephyr_library_sources_ifdef(CONFIG_MCTP_UART mctp_uart.c)
zephyr_library_sources_ifdef(CONFIG_MCTP_I3C mctp_i3c.c)

