/* This file was automatically generated.  Do not edit! */
bool TimecodeStringify(unsigned int inTimecode,unsigned int inTimecodeSubframe,char *Buffer,int BufferSize);
bool DecodeTimecode(unsigned int inTimecode,unsigned int inTimecodeSubframe,int *hour,int *minute,int *second,int *frame,int *subframe);
int main(int argc,char *argv[]);
SOCKET CreateCommandSocket(unsigned long IP_Address,unsigned short uPort);
DWORD WINAPI DataListenThread(void *dummy);
DWORD WINAPI CommandListenThread(void *dummy);
extern int gCommandResponseCode;
extern unsigned char gCommandResponseString[MAX_PATH];
extern int gCommandResponseSize;
extern int gCommandResponse;
extern int ServerVersion[4];
extern sockaddr_in HostAddr;
extern in_addr ServerAddress;
extern SOCKET DataSocket;
extern SOCKET CommandSocket;
int SendCommand(char *szCOmmand);
int SendCommand(char *szCommand);
int GetLocalIPAddresses(unsigned long Addresses[],int nMax);
int GetLocalIPAddresses(unsigned long Addresses[],int nMax);
void Unpack(char *pData);
void Unpack(char *pData);
bool IPAddress_StringToAddr(char *szNameOrAddress,struct in_addr *Address);
bool IPAddress_StringToAddr(char *szNameOrAddress,struct in_addr *Address);
extern int NatNetVersion[4];
extern MOCAP_RIGID_BODY body;
