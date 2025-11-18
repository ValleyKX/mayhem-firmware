set(app_name "drone_scanner")

set(sources
    ui_drone_scanner.cpp
)

register_external_app(${app_name} "${sources}")
