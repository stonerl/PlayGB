#include "rom.h"

#include <stdlib.h>
#include <string.h>

void gb_read_header(struct gb_header *out, char *header)
{
    memset(out, 0, sizeof(*out));
    out->old_licensee_code = header[0x4b];
    bool is_new = out->old_licensee_code == 0x33;
    for (int i = 0; i < 16; ++i)
    {
        char c = header[0x34 + i];
        if (c < 0)
            break;
        out->title[i] = c;
        out->title[i + 1] = 0;
    }
}