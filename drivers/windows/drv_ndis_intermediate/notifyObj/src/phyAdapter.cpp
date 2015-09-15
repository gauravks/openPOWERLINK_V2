/**
********************************************************************************
\file   phyAdapter.cpp

\brief  Implementation of CPhyAdapter class

TODO:

\ingroup module_notify_ndisim
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "stdafx.h"
#include "phyAdapter.h"

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

CPhyAdapter::CPhyAdapter(INetCfg* pInetCfg_p, const GUID& guidAdapter_p)
{
    TraceMsg(L"---->Constructor CPhyAdapter\n");
    this->pInetCfg = pInetCfg_p;
    this->pInetCfg->AddRef();

    CopyMemory(&guidAdapter, &guidAdapter_p, sizeof(GUID));
    TraceMsg(L"<----Constructor CPhyAdapter\n");
}

CPhyAdapter::~CPhyAdapter()
{
    TraceMsg(L"---->Destructor ~CPhyAdapter\n");
    if (this->pVethMiniport != NULL)
        delete this->pVethMiniport;

    ReleaseObj(this->pInetCfg);
    TraceMsg(L"<----Destructor ~CPhyAdapter\n");
}

HRESULT CPhyAdapter::Initialize(VOID)
{
    HKEY                hkeyAdapterGuid;
    WCHAR               szAdapterGuidKey[MAX_PATH + 1];
    WCHAR               szAdapterGuid[MAX_PATH + 1];
    LPWSTR              szMiniportList = NULL;
    LPWSTR              szMiniport = NULL;
    LPWSTR              szMiniportGuid = NULL;
    DWORD               dispostion;
    CVEthMiniport*      pMiniport = NULL;
    GUID                miniportGUID;
    DWORD               bytes;
    LONG                result;

    TraceMsg(L"--->CPhyAdapter::Initialize\n");

    StringFromGUID2(guidAdapter, szAdapterGuid, MAX_PATH + 1);
    TraceMsg(L"Adapter %s\n", szAdapterGuid);

    StringCchPrintf(szAdapterGuidKey,
                     celems(szAdapterGuidKey),
                     L"%s\\%s",
                     c_szAdapterList,
                     szAdapterGuid);


    szAdapterGuidKey[MAX_PATH] = '\0';
    TraceMsg(L"Miniport %s\n", szAdapterGuidKey);
    result = RegCreateKeyEx(HKEY_LOCAL_MACHINE,
                            szAdapterGuidKey,
                            0,
                            NULL,
                            REG_OPTION_NON_VOLATILE,
                            KEY_ALL_ACCESS,
                            NULL,
                            &hkeyAdapterGuid,
                            &dispostion);

    if (result != ERROR_SUCCESS)
        return HRESULT_FROM_WIN32(result);

    if (dispostion == REG_CREATED_NEW_KEY)
    {
        // New key created, no need to setup
        TraceMsg(L"New Key Created\n");
        goto Exit;
    }

    bytes = 0;
    result = RegQueryValueEx(hkeyAdapterGuid, // Registry key to query
                             c_szUpperBindings, // Name of the registry value
                             NULL,              // Reserved
                             NULL,              // Variable type
                             NULL,              // Data in the variable
                             &bytes);

    if (result != ERROR_SUCCESS)
    {
        TraceMsg(L"Error is here1 %d\n", result);
    }

    TraceMsg(L"Allocating variable %d\n", bytes);
    szMiniportList = (LPWSTR)calloc(bytes, 1);

    if (szMiniportList == NULL)
    {
        result = ERROR_NOT_ENOUGH_MEMORY;
        goto Exit;
    }

    result = RegQueryValueEx(hkeyAdapterGuid, // Registry key to query
                             c_szUpperBindings, // Name of the registry value
                             NULL,              // Reserved
                             NULL,              // Variable type
                             (LPBYTE) szMiniportList,              // Data in the variable
                             &bytes);

    if (result != ERROR_SUCCESS)
    {
        TraceMsg(L"Error is here %d %d\n", result, bytes);
        goto Exit;
    }

    szMiniport = szMiniportList;

    szMiniportGuid = RemoveDevicePrefix(szMiniport);

    TraceMsg(L"   Loading configuration for miniport %s...\n",
             szMiniportGuid);

    if (szMiniportGuid)
    {
        CLSIDFromString(szMiniportGuid, &miniportGUID);

        pMiniport = new CVEthMiniport(pInetCfg, &miniportGUID,
                                          &this->guidAdapter);

        if (pMiniport == NULL)
        {
            result = ERROR_NOT_ENOUGH_MEMORY;
            goto Exit;
        }

        // Initialize the miniport
        pMiniport->Initialize();

        // Store the pointer
        this->pVethMiniport = pMiniport;
    }

Exit:
    if (szMiniportGuid != NULL)
        free(szMiniportGuid);

    if (szMiniportList != NULL)
        free(szMiniportList);

    RegCloseKey(hkeyAdapterGuid);

    TraceMsg(L"<---CPhyAdapter::Initialize HRESULT %x \n", result);
    return HRESULT_FROM_WIN32(result);
}

VOID CPhyAdapter::GetAdapterGUID(GUID *guidAdapter)
{
    CopyMemory(guidAdapter, &this->guidAdapter, sizeof(GUID));
}

HRESULT CPhyAdapter::AddMiniport(CVEthMiniport *pNewMiniport)
{
    if (this->pVethMiniport != NULL)
        return HRESULT_FROM_WIN32(ERROR_ALREADY_ASSIGNED);

    this->pVethMiniport = pNewMiniport;
    this->pVethMiniport->SetConfigAction(kActAdd);

    return S_OK;
}

HRESULT CPhyAdapter::RemoveMiniport(VOID)
{
    TraceMsg(L"---->CPhyAdapter::RemoveMiniport\n");
    if (this->pVethMiniport == NULL)
        return HRESULT_FROM_WIN32(ERROR_INVALID_PARAMETER);

    this->pVethMiniport->DeInstall();
    this->pVethMiniport->SetConfigAction(kActRemove);
    TraceMsg(L"<----CPhyAdapter::RemoveMiniport\n");
    return S_OK;
}

HRESULT CPhyAdapter::ApplyRegistryChanges()
{
    HKEY                    hkeyAdapterList;
    HKEY                    hkeyAdapterGuid;
    WCHAR                   szAdapterGuid[MAX_PATH + 1];
    CVEthMiniport*          pMiniport = NULL;
    DWORD                   miniportCount;
    DWORD                   disposition;
    DWORD                   i;
    LONG                    result;
    HRESULT                 hret;

    TraceMsg(L"---->CPhyAdapter::ApplyRegistryChanges.");

    if (this->applyAction == kActRemove)
        TraceMsg(L"Remove\n");
    else
        if (this->applyAction == kActAdd)
            TraceMsg(L"Add\n");
        else
            if (this->applyAction == kActUpdate)
                TraceMsg(L"Update\n");

    StringFromGUID2(this->guidAdapter,
                    szAdapterGuid,
                    MAX_PATH + 1);

    result = RegCreateKeyEx(HKEY_LOCAL_MACHINE,
                              c_szAdapterList,
                              0,
                              NULL,
                              REG_OPTION_NON_VOLATILE,
                              KEY_ALL_ACCESS,
                              NULL,
                              &hkeyAdapterList,
                              &disposition);

    if (result == ERROR_SUCCESS)
    {
        result = RegCreateKeyEx(hkeyAdapterList,
                                szAdapterGuid,
                                0,
                                NULL,
                                REG_OPTION_NON_VOLATILE,
                                KEY_ALL_ACCESS,
                                NULL,
                                &hkeyAdapterGuid,
                                &disposition);

        if (result == ERROR_SUCCESS)
        {
            RegCloseKey(hkeyAdapterGuid);
        }
        else
        {
            TraceMsg(L"   Failed to Create/Open the registry key: %s\\%s.\n",
                     c_szAdapterList, szAdapterGuid);
        }

    }
    else
    {
        TraceMsg(L"   Failed to open the registry key: %s.\n",
                 c_szAdapterList);
        return S_OK;
    }

    this->pVethMiniport->ApplyRegistryChanges();

    // Delete the registry key if adapter is being removed

    if (this->applyAction == kActRemove)
    {
        // Delete the key
        RegDeleteKey(hkeyAdapterList, szAdapterGuid);
        RegCloseKey(hkeyAdapterList);
    }
    TraceMsg(L"<----CPhyAdapter::ApplyRegistryChanges.");
    return S_OK;
}

HRESULT CPhyAdapter::SetConfigAction(tConfigAction applyAction_p)
{
    this->applyAction = applyAction_p;
    //TODO: Test this change
    if (this->pVethMiniport != NULL)
        this->pVethMiniport->SetConfigAction(applyAction_p);

    return S_OK;
}

HRESULT CPhyAdapter::ApplyPnpChanges(INetCfgPnpReconfigCallback *pfnCallback)
{
    CVEthMiniport*      pMiniport = this->pVethMiniport;

    TraceMsg(L"-----> CPhyAdapter::ApplyPnpChanges....");

    if (this->applyAction == kActRemove)
    {
        TraceMsg(L"kActRemove\n");
        pMiniport->SetConfigAction(kActRemove);
        pMiniport->ApplyPnpChanges(pfnCallback);
        delete pMiniport;
        this->pVethMiniport = NULL;
    }

    if (this->applyAction == kActAdd)
    {
        TraceMsg(L"kActAdd\n");
        pMiniport->SetConfigAction(kActAdd);
        pMiniport->ApplyPnpChanges(pfnCallback);
    }

    if (this->applyAction == kActUpdate)
    {
        TraceMsg(L"kActUpdate\n");
        pMiniport->ApplyPnpChanges(pfnCallback);
    }

    TraceMsg(L"<-----CPhyAdapter::ApplyPnpChanges....");
    return S_OK;
}

HRESULT CPhyAdapter::CancelChanges(VOID)
{
    return S_OK;
}

BOOL CPhyAdapter::MiniportPresent(VOID)
{
    if (this->pVethMiniport != NULL)
        return TRUE;

    return FALSE;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
