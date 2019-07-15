
void writeRegister(unsigned char r, unsigned char v)
{
	Wire.beginTransmission(I2C_ADDRESS);
	Wire.write(r);
	Wire.write(v);
	Wire.endTransmission();
}


unsigned char readRegister(unsigned char r)
{
	unsigned char v;
	
	Wire.beginTransmission(I2C_ADDRESS);
	Wire.write(r); 								// Register to read
	Wire.endTransmission();
	
	Wire.requestFrom(I2C_ADDRESS, 1);			// Read a byte
	while(Wire.available() == 0)
	{
		//Wait
	}
	
	v = Wire.read();
	return v;
}

unsigned long readLong(unsigned char r)
{
	union
	{
		char data[4];
		unsigned long value;
	}
	 
	byteMappedLong;
	byteMappedLong.value = 0L;
	
	Wire.beginTransmission(I2C_ADDRESS);			// Begin read cycle
	Wire.write(0);									// Pointer to first data register
	Wire.endTransmission();							// End cycle
	
	//The data pointer is reset anyway - so read from 0 on

	Wire.requestFrom(I2C_ADDRESS,r+4); 				// Read 2 bytes plus all bytes before the register 
	
	while (!Wire.available()==r+4)
	{
		//Wait
	}

	for (int i=r+3; i>=0; i--)
	{
		uint8_t c = Wire.read();
		if (i<4)
		{
			byteMappedLong.data[i]= c;
		}
	}
	
	return byteMappedLong.value;
}
