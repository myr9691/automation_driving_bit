#define button 29

void power_setup()
{
	pinMode(button, INPUT);
}

int power_on_off()
{
	return digitalRead(button);
}
