#include <stdint.h>
#include <stdbool.h>

struct gb_header
{
    char old_licensee_code;
    union
    {
        char title[17];
        struct {
            char _[11];
            uint32_t manufacturer;
            char cgb;
        } __attribute__((packed));
    };
    
    uint16_t new_licensee_code;
    
    // TODO: other flags
};

void gb_read_header(struct gb_header* out, char* header);