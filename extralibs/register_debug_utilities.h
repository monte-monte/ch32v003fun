//! ####################################
//! PRINT BITS
//! ####################################

#define UTIL_PRINT_BITS(val, len, separator, divider_len) \
	printf("["); \
	for (int i = len-1; i >= 0; i--) { \
		printf("%d", ((val) >> i) & 1); if (i > 0) printf(i%divider_len ? " " : " %s ", separator); \
	}; printf("]\n");

// #define UTIL_PRINT_BITS8(val, ...) \
//     struct { const char* sep; int group; } params = {"|", 8}; \
//     void *args[] = {__VA_ARGS__}; \
//     int arg_count = sizeof(args) / sizeof(args[0]); \
//     if (arg_count >= 1) params.sep = (const char*)args[0]; \
//     if (arg_count >= 2) params.group = *(int*)args[1]; \
//     UTIL_PRINT_BITS(val, 8, params.sep, params.group);


#define UTIL_PRINT_BITS8(val) UTIL_PRINT_BITS(val, 8, "|", 8)
#define UTIL_PRINT_BITS16(val) UTIL_PRINT_BITS(val, 16, "|", 8)
#define UTIL_PRINT_BITS32(val) UTIL_PRINT_BITS(val, 32, "|", 8)


// eg: UTIL_PRINT_BIT_PAIRS(reg, {"a", 0}, {"b", 1}, {"c", 2}, {"d", 3});
#define UTIL_PRINT_BIT_PAIRS(reg, ...) \
    struct BitPair { const char* name; int pos; }; \
    struct BitPair pairs[] = {__VA_ARGS__}; \
    for (int i = 0; i < sizeof(pairs) / sizeof(pairs[0]); i++) { \
        if (i > 0) printf(", "); printf("%s=%d", pairs[i].name, (reg >> pairs[i].pos) & 1); }


// eg: UTIL_PRINT_BIT_RANGE(reg, {"field1", 0, 3}, {"field2", 4, 7});
#define UTIL_PRINT_BIT_RANGE(reg, ...) \
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
    }; printf("\n");


//! ####################################
//! PRINT REGS
//! ####################################

#define UTIL_PRINT_REG8(reg, label) printf("%s: 0x%02X\n", label, reg); UTIL_PRINT_BITS8(reg);
#define UTIL_PRINT_REG16(reg, label) printf("%s: 0x%04X\n", label, reg); UTIL_PRINT_BITS16(reg);
#define UTIL_PRINT_REG32(reg, label) printf("%s: 0x%08lX\n", label, reg); UTIL_PRINT_BITS32(reg);


