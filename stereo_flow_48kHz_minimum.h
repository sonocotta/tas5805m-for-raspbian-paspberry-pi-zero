#include <linux/regmap.h>

static const struct reg_sequence tas5805m_init_sequence[] = 
{
    { 0x00, 0x00 },
    { 0x7f, 0x00 },
    { 0x54, 0x03 },
    { 0x78, 0x80 },

};
