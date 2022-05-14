set(component_srcs
    "src/bk4802.c"
)

idf_component_register(SRCS "${component_srcs}"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS ""
    PRIV_REQUIRES "driver"
    REQUIRES ""
)
