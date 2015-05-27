// notifyObj.cpp : Implementation of DLL Exports.


#include "stdafx.h"
#include "resource.h"
#include "notifyObj_i.h"
#include "dllmain.h"
#include <netcfgn.h>
#include "common.h"

using namespace ATL;

// Used to determine whether the DLL can be unloaded by OLE.
STDAPI DllCanUnloadNow(void)
{
    HRESULT hr;

    TraceMsg(L"-->DllCanUnloadNow.\n");

    hr = (_AtlModule.GetLockCount() == 0) ? S_OK : S_FALSE;

    TraceMsg(L"-->DllCanUnloadNow(HRESULT = %x).\n",
             hr);

    return hr;
}

// Returns a class factory to create an object of the requested type.
STDAPI DllGetClassObject(_In_ REFCLSID rclsid, _In_ REFIID riid, _Outptr_ LPVOID* ppv)
{
    TraceMsg(L"DllGetClassObject.\n");
    return _AtlModule.DllGetClassObject(rclsid, riid, ppv);
}

// DllRegisterServer - Adds entries to the system registry.
STDAPI DllRegisterServer(void)
{
    // registers object, typelib and all interfaces in typelib
    TraceMsg(L"DllRegisterServer.\n");
    HRESULT hr = _AtlModule.DllRegisterServer(TRUE);

    TraceMsg(L"DllRegisterServer (HRESULT = %x).\n", hr);
    return hr;
}

// DllUnregisterServer - Removes entries from the system registry.
STDAPI DllUnregisterServer(void)
{
    HRESULT hr = _AtlModule.DllUnregisterServer();
    TraceMsg(L"DllUnregisterServer.\n");
    return hr;
}

// DllInstall - Adds/Removes entries to the system registry per user per machine.
STDAPI DllInstall(BOOL bInstall, _In_opt_  LPCWSTR pszCmdLine)
{
    HRESULT hr = E_FAIL;
    static const wchar_t szUserSwitch[] = L"user";
    TraceMsg(L"DllInstall.\n");
    if (pszCmdLine != NULL)
    {
        if (_wcsnicmp(pszCmdLine, szUserSwitch, _countof(szUserSwitch)) == 0)
        {
            ATL::AtlSetPerUserRegistration(true);
        }
    }

    if (bInstall)
    {
        hr = DllRegisterServer();
        if (FAILED(hr))
        {
            DllUnregisterServer();
        }
    }
    else
    {
        hr = DllUnregisterServer();
    }

    return hr;
}


