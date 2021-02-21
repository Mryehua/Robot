#include <QCoreApplication>
#include"pci1711.h"
#include<iostream>
#include <windows.h>
using namespace std;

int main(int argc, char *argv[])
{

    pci1711 pci;
    pci.initAO(0);
    pci.AOdataWrite(1,2);
    cout<<"the AOdata is "<<2<<"V";
    //pci.Initialize();
   // cout<<pci.initAO(1);
    /*pci.initAO(1);
     pci.aoData=0;
     pci.AOdata(0);*/
    /*pci.initDO(1);
    pci.buffereForWriting=0x01;
    pci.DOdata(0);*/
    return 0;
}
