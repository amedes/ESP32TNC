set(component_srcs
    "src/BME280.c"
    "src/ax25.c"
    )

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "include" "../../main"
                       PRIV_INCLUDE_DIRS ""
                       PRIV_REQUIRES "driver"
                       REQUIRES "m5atom"
		       )
