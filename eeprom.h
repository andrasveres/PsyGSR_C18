

void WriteLongEEPROM(unsigned long addr, unsigned long i);
int ReadLongEEPROM(unsigned long addr, unsigned long *i);
int ReadCharEEPROM(unsigned long addr, unsigned char *c);

int WriteEEPROM(unsigned long addr, unsigned char data);
int ReadEEPROM(unsigned long addr, unsigned char *data, unsigned char length);
void PageClearEEPROM(unsigned long addr);

