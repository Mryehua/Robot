#include "pci1711.h"

pci1711::pci1711()
{
}
/*************初始化函数***************/

void pci1711::initAI(int id)
{
    ErrorCode        ret = Success;
    instantAiCtrl = InstantAiCtrl::Create();
    DeviceInformation devInfo(id);//deviceDescription
    ret = instantAiCtrl->setSelectedDevice(devInfo);
}
double pci1711::initAO(int id)
{
    ErrorCode  ret = Success;
    instantAoCtrl = InstantAoCtrl::Create();
    DeviceInformation devInfo(id);//deviceDescription
    ret = instantAoCtrl->setSelectedDevice(devInfo);
    instantAoCtrl->getChannels()->getItem(1).setValueRange(V_0To10);

}

/*void pci1711::initDI(int id)
{
    ErrorCode        ret = Success;
    instantDiCtrl = InstantDiCtrl::Create();
    DeviceInformation devInfo(id);
    ret = instantDiCtrl->setSelectedDevice(devInfo);
}*/
/*
void pci1711::initDO(int id)
{
    ErrorCode        ret = Success; 
    instantDoCtrl = InstantDoCtrl::Create();
    DeviceInformation devInfo(id);//deviceDescription
    ret = instantDoCtrl->setSelectedDevice(devInfo);
}
*/

/***********读写函数************/
double pci1711::AIdata(int countID)
{
    ErrorCode ret = Success;
    double scaledData;
    ret=instantAiCtrl->Read(countID, 1, &scaledData);
    return scaledData;
}
void pci1711::AOdataWrite(int countID,double aoData)
{
    ErrorCode  ret = Success;

    ret = instantAoCtrl->Write(countID,aoData);
}

/*************************************************************/
/*
double pci1711::high()
{
    ErrorCode  ret = Success;
    double bufferForReading;
    ret = instantDiCtrl->Read(0, 1, bufferForReading);
    return bufferForReading;
}
double pci1711::go()
{
    ErrorCode  ret = Success;
    double bufferForReading1;
    ret = instantDiCtrl->Read(1, 1, bufferForReading1);
    return bufferForReading1;
}
double pci1711::low()
{
    ErrorCode  ret = Success;
    double bufferForReading2;
    ret = instantDiCtrl->Read(2, 1, bufferForReading2);
    return bufferForReading2;
}
*/
/**************************************************************/
/*
void pci1711::DOdata(int countID,buffereForWriting)
{
    ErrorCode        ret = Success;
    //byte  bufferForWriting[64] = {0};//the first element is used for start port
    ret=instantDoCtrl->Write(countID,1,&buffereForWriting);
}

*/

/*************结束函数**********/
void pci1711::AIfinish()
{
    // Step 4 : Close device and release any allocated resource.
    instantAiCtrl->Dispose();
}
void pci1711::AOfinish()
{
    // Step 4: Close device and release any allocated resource.
    instantAoCtrl->Dispose();
}
void pci1711::DOfinish()
{
    // Step 4: Close device and release any allocated resource.
    instantDoCtrl->Dispose();
}
