#ifndef COMMON_H_INCLUDED

#define COMMON_H_INCLUDED

#include <devguid.h>
#include <strsafe.h>

#define celems(_x)          (sizeof(_x) / sizeof(_x[0]))

#define     MAX_VIRTUAL_MP_PER_ADAPTER      1
#define DEBUGL 1
typedef enum
{
    kActUnknown,
    kActInstall,
    kActAdd,
    kActRemove,
    kActUpdate,
    kActPropertyUIAdd,
    kActPropertyUIRemove
} eConfigAction;

typedef char tConfigAction;


//
// PnP ID, also referred to as Hardware ID, of the protocol interface.
//

const WCHAR c_szMuxProtocol[] = {L"ms_oplkp"};

//
// PnP ID, also referred to as Hardware ID, of the Miniport interface.
//

const WCHAR c_szMuxMiniport[] = {L"ms_oplkmp"};

//
// Name of the service as specified in the inf file in AddService directive.
//

const WCHAR c_szMuxService[] = {L"oplkp"};

//
// Path to the config string where the virtual miniport instance names
// are stored.
//

const WCHAR c_szAdapterList[] =
{L"System\\CurrentControlSet\\Services\\oplkp\\Parameters\\Adapters"};

//
// Value name in the registry where miniport device id is stored.
//

const WCHAR c_szUpperBindings[] = {L"UpperBindings"};


const WCHAR c_szDevicePrefix[] = {L"\\Device\\"};

#define ReleaseObj( x )  if ( x ) \
                            ((IUnknown*)(x))->Release();


#if DEBUGL
void TraceMsg(_In_ LPWSTR szFormat, ...);
void DumpChangeFlag(DWORD dwChangeFlag);
void DumpBindingPath(INetCfgBindingPath* pncbp);
void DumpComponent(INetCfgComponent *pncc);
#else
//TODO: Remove it after testing
//void TraceMsg(_In_ LPWSTR szFormat, ...);
//void DumpChangeFlag(DWORD dwChangeFlag);
//void DumpBindingPath(INetCfgBindingPath* pncbp);
//void DumpComponent(INetCfgComponent *pncc);

#define TraceMsg
#define DumpChangeFlag
#define DumpBindingPath
#define DumpComponent
#endif

// TODO:clean this up
HRESULT FindInstance(INetCfg *pnc,
                       GUID &guidInstance,
                       INetCfgComponent **ppnccMiniport);

LONG AddToMultiSzValue(HKEY hkeyAdapterGuid,
                       _In_ LPWSTR szMiniportGuid);

LONG DeleteFromMultiSzValue(HKEY hkeyAdapterGuid,
                            _In_ LPWSTR szMiniportGuid);

LPWSTR AddDevicePrefix(_In_ LPWSTR lpStr);
LPWSTR RemoveDevicePrefix(_In_ LPWSTR lpStr);

#endif // COMMON_H_INCLUDED