#pragma once

#include <windows.h>
#include <netcfgn.h>

#include "common.h"
#include "vethMiniport.h"

class CPhyAdapter
{

    GUID            guidAdapter;
    INetCfg*        pInetCfg;
    CVEthMiniport*  pVethMiniport;
    BOOL            fActive;
    tConfigAction   applyAction;

public:
    CPhyAdapter(INetCfg* pInetCfg, const GUID& pGuidAdapter);
    ~CPhyAdapter();
    HRESULT Initialize(VOID);
    VOID    GetAdapterGUID(GUID *guidAdapter);
    HRESULT AddMiniport(CVEthMiniport *pNewMiniport);
    HRESULT RemoveMiniport(VOID);
    HRESULT ApplyRegistryChanges(void);
    HRESULT SetConfigAction(tConfigAction applyAction_p);
    HRESULT ApplyPnpChanges(INetCfgPnpReconfigCallback *pfnCallback);
    HRESULT CancelChanges(VOID);
    BOOL    MiniportPresent(VOID);
};

