

#include "Translate_Result.h"

//void printServoId(String msg)
//{
//  print(msg);
//  print(" servo ID ");
//  print(_id);
//  print(" - ");
//}

void printDxlResult()
{
   while(!dxlCom.dxlDataReady());        // waiting the answer of servo
   printDxlError(dxlCom.readDxlError());
   Serial.println(dxlCom.readDxlResult());
}


void printDxlError(unsigned short dxlError)
{
  // after any operation error can be retrieve using dx::readDxlResult() (i.e. after read or write operation)
  if(dxlError == DXL_ERR_SUCCESS)
    printf("OK");
  else
  {
    if (dxlError & DXL_ERR_VOLTAGE)
      print("voltage out of range-");
    if (dxlError & DXL_ERR_ANGLE)
      print("angle out of range-");
    if (dxlError & DXL_ERR_OVERHEATING)
      print("overheating-");
    if (dxlError & DXL_ERR_RANGE)
      print("cmd out of range-");
    if (dxlError & DXL_ERR_TX_CHECKSUM)
      print("Tx CRC invalid-");
    if (dxlError & DXL_ERR_OVERLOAD )
      print("overload-");
    if (dxlError & DXL_ERR_INSTRUCTION )
      print("undefined instruction-");
    if (dxlError & DXL_ERR_TX_FAIL )
      print("Tx No header-");
    if (dxlError & DXL_ERR_RX_FAIL )
      print("Rx No header-");
    if (dxlError & DXL_ERR_TX_ERROR  )
      print("Tx error-");
    if (dxlError & DXL_ERR_RX_LENGTH   )
      print("Rx length invalid-");  // Not implemented yet
    if (dxlError & DXL_ERR_RX_TIMEOUT)
      print("timeout-");
    if (dxlError & DXL_ERR_RX_CORRUPT)
      print("Rx CRC invalid-");
    if (dxlError & DXL_ERR_ID )
      print("Wrong ID answered-"); // Hardware issue
  }
}