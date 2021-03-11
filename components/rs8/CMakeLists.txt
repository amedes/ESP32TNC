set(component_srcs
    "src/gf8.c"
    "src/rs8.c"
    "src/poly8.c"
    "src/rs8_gen_poly.c"
    )

idf_component_register(SRCS "${component_srcs}"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS ""
    PRIV_REQUIRES ""
    REQUIRES ""
    )
