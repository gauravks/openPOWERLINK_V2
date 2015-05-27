// dllmain.cpp : Implementation of DllMain.

#include "stdafx.h"
#include "resource.h"
#include "notifyObj_i.h"
#include "dllmain.h"
#include "notify.h"
#include <netcfgn.h>
#include "common.h"

CnotifyObjModule _AtlModule;

BEGIN_OBJECT_MAP(ObjectMap)
    OBJECT_ENTRY(CLSID_CNotify, CNotify)
END_OBJECT_MAP()

// DLL Entry Point
extern "C" BOOL WINAPI DllMain(HINSTANCE hInstance, DWORD dwReason, LPVOID lpReserved)
{
    hInstance;

    TraceMsg(L"DllMain.\n");
    if (dwReason == DLL_PROCESS_ATTACH)
    {
        TraceMsg(L"   Reason: Attach.\n");
        DisableThreadLibraryCalls(hInstance);
    }
    else
    {
        if (dwReason == DLL_PROCESS_DETACH)
        {
            TraceMsg(L"   Reason: Detach.\n");
        }
    }

    return _AtlModule.DllMain(dwReason, lpReserved);
}
