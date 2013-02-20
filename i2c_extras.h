unsigned char LDByteWriteI2C( unsigned char ControlByte, unsigned char LowAdd, unsigned char data );
unsigned char LDByteReadI2C( unsigned char ControlByte, unsigned char address, unsigned char *data, unsigned char length );
unsigned char LDPageWriteI2C( unsigned char ControlByte, unsigned char LowAdd, unsigned char *wrptr );
unsigned char LDSequentialReadI2C( unsigned char ControlByte, unsigned char address, unsigned char *rdptr, unsigned char length );
unsigned char HDByteWriteI2C( unsigned char ControlByte, unsigned char HighAdd, unsigned char LowAdd, unsigned char data );
unsigned char HDByteReadI2C( unsigned char ControlByte, unsigned char HighAdd, unsigned char LowAdd, unsigned char *data, unsigned char length );
unsigned char HDPageWriteI2C( unsigned char ControlByte, unsigned char HighAdd, unsigned char LowAdd, unsigned char *wrptr );
unsigned char HDSequentialReadI2C( unsigned char ControlByte, unsigned char HighAdd, unsigned char LowAdd, unsigned char *rdptr, unsigned char length );
unsigned char putstringI2C( unsigned char *wrptr );

