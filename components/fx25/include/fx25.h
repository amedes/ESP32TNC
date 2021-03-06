
#define FX25_CODE_MAX   255
#define FX25_PARITY_MIN 16
#define FX25_DATA_LEN_MAX   (FX25_CODE_MAX - FX25_PARITY_MIN)

typedef struct FX25TAG {
    uint64_t cotag;
    int rs_code;
    int rs_info;
} fx25tag_t;

extern const fx25tag_t fx25tag[];

enum FX25TAG_NO {
    TAG_00 = 0,
    TAG_01,
    TAG_02,
    TAG_03,
    TAG_04,
    TAG_05,
    TAG_06,
    TAG_07,
    TAG_08,
    TAG_09,
    TAG_0A,
    TAG_0B,
    TAG_0C,
    TAG_0D,
    TAG_0E,
    TAG_0F,
};
