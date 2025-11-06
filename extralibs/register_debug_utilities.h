//! ####################################
//! PRINT BITS
//! ####################################

// last 2 args are optional: separator string (default "|"), divider length (default 8)
#define UTIL_PRINT_BITS(val, len, ...) do { \
    const char* _sep = "|"; \
    int _div = 8; \
    const void* _args[] = {__VA_ARGS__}; \
    int _cnt = sizeof(_args) / sizeof(_args[0]); \
    if (_cnt > 0) _sep = (const char*)_args[0]; \
    if (_cnt > 1) _div = (int)(size_t)_args[1]; \
    printf("["); \
    for (int _i = (len)-1; _i >= 0; _i--) { \
        printf("%d", ((val) >> _i) & 1); \
        if (_i > 0) printf(_i % _div ? " " : " %s ", _sep); \
    } \
    printf("]\n"); \
} while(0)

#define UTIL_PRINT_BITS8(val, ...) UTIL_PRINT_BITS(val, 8, __VA_ARGS__)
#define UTIL_PRINT_BITS16(val, ...) UTIL_PRINT_BITS(val, 16, __VA_ARGS__)
#define UTIL_PRINT_BITS32(val, ...) UTIL_PRINT_BITS(val, 32, __VA_ARGS__)


// eg: UTIL_PRINT_BIT_PAIRS(reg, {"a", 0}, {"b", 1}, {"c", 2}, {"d", 3});
#define UTIL_PRINT_BIT_PAIRS(reg, ...) do { \
	struct BitPair { const char* name; int pos; }; \
	struct BitPair pairs[] = {__VA_ARGS__}; \
	for (int i = 0; i < sizeof(pairs) / sizeof(pairs[0]); i++) { \
		if (i > 0) printf(", "); printf("%s=%d", pairs[i].name, (reg >> pairs[i].pos) & 1); } \
} while(0)

// eg: UTIL_PRINT_BIT_RANGE(reg, {"field1", 0, 3}, {"field2", 4, 7});
#define UTIL_PRINT_BIT_RANGE(reg, ...) do { \
	struct BitField { const char* name; int start; int end; }; \
	struct BitField fields[] = {__VA_ARGS__}; \
	for (int i = 0; i < sizeof(fields) / sizeof(fields[0]); i++) { \
		if (i > 0) printf(", "); \
		if (fields[i].end == fields[i].start) { \
			/* Single bit */ \
			printf("%s=%d", fields[i].name, (reg >> fields[i].start) & 1); \
		} else { \
			/* Multiple bits */ \
			int mask = ((1 << (fields[i].end - fields[i].start + 1)) - 1); \
			printf("%s=0x%02X", fields[i].name, (reg >> fields[i].start) & mask); \
		} \
	}; printf("\n"); \
} while(0)

//! ####################################
//! PRINT REGS
//! ####################################

#define UTIL_PRINT_REG8(reg, label) printf("%s: 0x%02X\n", label, reg);
#define UTIL_PRINT_REG16(reg, label) printf("%s: 0x%04X\n", label, reg);
#define UTIL_PRINT_REG32(reg, label) printf("%s: 0x%08lX\n", label, reg);


