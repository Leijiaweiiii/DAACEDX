#include "DAACEDfont.h"
#include "DAACEDbitmap.h"

const uint8_t bt_bitmap[] ={
    0x0C, 0x03, 0x00, //     ##        ##
    0x06, 0x06, 0x00, //      ##      ##
    0x03, 0x0C, 0x00, //       ##    ##
    0x01, 0x98, 0x00, //        ##  ##
    0x00, 0xF0, 0x00, //         ####
    0x7F, 0xFF, 0xE0, //  ##################
    0xFF, 0xFF, 0xF0, // ####################
    0xE0, 0xF0, 0x70, // ###     ####     ###
    0x71, 0xF8, 0xE0, //  ###   ######   ###
    0x3B, 0x9D, 0xC0, //   ### ###  ### ###
    0x1F, 0x0F, 0x80, //    #####    #####
    0x0E, 0x07, 0x00, //     ###      ###
    0x04, 0x02, 0x00, //      #        #
};
const bitmap_data_t bt_bitmap_data = {bt_bitmap, 13, 3};

const uint8_t battery_right[] ={
    0xC0, 0x00, 0x30, // ##                ##
    0x40, 0x00, 0x20, //  #                #
    0x7F, 0xFF, 0xE0, //  ##################
    0x3F, 0xFF, 0x80, //   ################
};
const bitmap_data_t battery_right_bitmap = {battery_right, 4, 3};
const uint8_t battery_middle_full[] ={
    0xDF, 0xFF, 0xB0, // ## ############## ##
};
const bitmap_data_t battery_middle_full_bitmap = {battery_middle_full, 1, 3};

const uint8_t battery_middle_empty[] ={
    0xC0, 0x00, 0x30, // ##                ##
};
const bitmap_data_t battery_middle_empty_bitmap = {battery_middle_empty, 1, 3};
const uint8_t battery_left[] ={
    0x03, 0xFC, 0x00, //       ########
    0x06, 0x06, 0x00, //      ##      ##
    0x3F, 0xFF, 0xC0, //   ################
    0x60, 0x00, 0x60, //  ##              ##
    0xC0, 0x00, 0x30, // ##                ##
};
const bitmap_data_t battery_left_bitmap = {battery_left, 5, 3};
