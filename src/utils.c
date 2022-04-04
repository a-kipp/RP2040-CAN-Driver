uint16_t utils_byteswap16(uint16_t num) {
    uint16_t swapped = (num>>8) | (num<<8);
    return swapped;
}

uint32_t utils_byteswap32(uint32_t num) {
    uint32_t swapped = ((num>>24)&0xff) | // move byte 3 to byte 0
                       ((num<<8)&0xff0000) | // move byte 1 to byte 2
                       ((num>>8)&0xff00) | // move byte 2 to byte 1
                       ((num<<24)&0xff000000); // byte 0 to byte 3
    return swapped;
}