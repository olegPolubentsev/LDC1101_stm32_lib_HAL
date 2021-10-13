
#include "main.h"
#include "LDC1101.h"

uint32_t val = 0;

int main(void)
{
  
  defInitConfigStructFDC FDC2112_A;
  FDC2112_Init(&hi2c1, FDC2214_I2C_ADDR_0, &FDC2112_A);
  FDC2112_Eneble_Convertion(&FDC2112_A);
 
  while (1)
  {
	  val = FDC2112_Get_Value_Chanel_A(&FDC2112_A);
  }
}

