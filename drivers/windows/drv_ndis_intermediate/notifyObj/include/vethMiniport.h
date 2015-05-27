#pragma once

#include <windows.h>
#include <stdio.h>
#include "netcfgn.h"

#include "common.h"

class CVEthMiniport
{
    INetCfg*        pInetCfg;
    GUID            guidAdapter;
    GUID            guidMiniport;
    tConfigAction   applyAction;

public:
    CVEthMiniport(INetCfg* pInetCfg, GUID* pGuidMiniport, GUID* pGuidAdapter);
    ~CVEthMiniport();
    HRESULT Initialize(VOID);
    VOID    GetAdapterGUID(GUID*);
    VOID    GetMiniportGUID(GUID*);
    HRESULT Install(VOID);
    HRESULT DeInstall(VOID);
    HRESULT SetConfigAction(tConfigAction applyAction_p);
    HRESULT ApplyRegistryChanges(VOID);
    HRESULT ApplyPnpChanges(INetCfgPnpReconfigCallback* pfnCallback);
};

