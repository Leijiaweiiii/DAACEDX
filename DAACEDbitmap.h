
// How to convert bitmap to binary data?
// > Use "TheDotFactory.exe" utility.

typedef struct  {
    uint8_t * image_data;
    uint8_t width_in_bits;
    uint8_t heigth_in_bytes;
} bitmap_data_t;
extern const bitmap_data_t bt_bitmap_data;
extern const bitmap_data_t battery_right_bitmap;
extern const bitmap_data_t battery_middle_full_bitmap;
extern const bitmap_data_t battery_middle_empty_bitmap;
extern const bitmap_data_t battery_left_bitmap;
//extern const bitmap_data_t daaced_logo;
