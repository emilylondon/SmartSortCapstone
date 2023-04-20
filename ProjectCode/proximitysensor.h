#define PROX_ADDR 0x13
#define PROX_ADDR_W 0x26
#define PROX_ADDR_R 0x27
#define COMMAND_REG 0x80
#define PROX_RESULT_HIGH_REG 0x85
#define PROX_RESULT_LOW_REG 0x86
#define FOSC 9830400
#define BDIV (FOSC/ 100000 - 16) / 2 + 1

extern void i2c_init(uint8_t);
extern uint8_t i2c_io(uint8_t, uint8_t *, uint16_t,
		      uint8_t *, uint16_t, uint8_t *, uint16_t);
void init_proximity(void); 
char proximity_read(void);