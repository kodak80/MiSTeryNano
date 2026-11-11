// link stubs: the bridge (real drive) side of disk_hfe.cpp is not used here
#include <cstddef>
void *FloppyBridgeOpen(const char*,const char*,char*,int){return NULL;}
void FloppyBridgeClose(void*){}
int FloppyBridgeMaxCylinder(void*){return 0;}
bool FloppyBridgeDiskInDrive(void*){return false;}
void FloppyBridgeMotorOff(void*){}
int g_fdc_cmd=0;
bool FloppyBridgeCanWrite(void*){return false;}
int FloppyBridgeWriteTrack(void*,int,int,const unsigned char*,int,int){return -1;}
int floppy_mediach[2]={0,0};
void FloppyBridgeLogMessage(void*,const char*){}
void FloppyBridgeForget(void*){}
bool FloppyBridgeWriteProtected(void*){return true;}
bool FloppyBridgeSameTrack(const unsigned char*,int,const unsigned char*,int){return true;}
int FloppyBridgeReadTrack(void*,int,int,unsigned char*,unsigned short*,int,unsigned char*){return -1;}
