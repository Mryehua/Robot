#ifndef PCI1711_H
#define PCI1711_H

#include <stdlib.h>
#include <stdio.h>
#include "C:/Advantech/DAQNavi/Examples/C++_Console/inc/compatibility.h"
#include "C:/Advantech/DAQNavi/Inc/bdaqctrl.h"


using namespace Automation::BDaq;

class pci1711
{
public:
    pci1711();
    ErrorCode  ret;

    InstantAiCtrl * instantAiCtrl;
    InstantAoCtrl * instantAoCtrl;
    InstantDoCtrl * instantDiCtrl;
    InstantDoCtrl * instantDoCtrl;

    void initAI(int id);
    double initAO(int id);
    void initDO(int id);

    double AIdata(int countID);
    void AOdataWrite(int countID,double aoData);
/*
    double high();
    double low();
    double go();
*/
   // void DOdata(int countID,buffereForWriting);

    void AIfinish();
    void AOfinish();
    void DIfinish();
    void DOfinish();
};

#endif // PCI1711_H
