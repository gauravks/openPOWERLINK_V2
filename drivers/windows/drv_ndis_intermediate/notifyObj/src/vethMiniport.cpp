/**
********************************************************************************
\file   vethMiniport.cpp

\brief  Implementation of CVethMiniport class

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
#include "vethMiniport.h"

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

CVEthMiniport::CVEthMiniport(INetCfg* pInetCfg_p, GUID* pGuidMiniport_p, GUID* pGuidAdapter_p)
{
    TraceMsg(L"---->Constructor CVEthMiniport.\n");

    this->pInetCfg = pInetCfg_p;
    this->pInetCfg->AddRef();

    CopyMemory(&this->guidAdapter, pGuidAdapter_p, sizeof(GUID));
    
    if (pGuidMiniport_p)
        CopyMemory(&this->guidMiniport, pGuidMiniport_p, sizeof(GUID));
    else
        ZeroMemory(&this->guidMiniport, sizeof(GUID));

    this->applyAction = kActUnknown;

    TraceMsg(L"<----Constructor CVEthMiniport.\n");
}

CVEthMiniport::~CVEthMiniport()
{
    TraceMsg(L"--->~CVEthMiniport.\n");
    ReleaseObj(this->pInetCfg);
    TraceMsg(L"<---~CVEthMiniport.\n");
}

HRESULT CVEthMiniport::Initialize(VOID)
{
    TraceMsg(L"Initialize.\n");
    return S_OK;
}

VOID CVEthMiniport::GetAdapterGUID(GUID* pGuidAdapter_p)
{
    TraceMsg(L"GetAdapterGUID.\n");
    if (pGuidAdapter_p)
        CopyMemory(pGuidAdapter_p, &this->guidAdapter, sizeof(GUID));

}

VOID CVEthMiniport::GetMiniportGUID(GUID* pGuidMiniport_p)
{
    TraceMsg(L"GetMiniportGUID.\n");
    if (pGuidMiniport_p)
        CopyMemory(pGuidMiniport_p, &this->guidMiniport, sizeof(GUID));
}

HRESULT CVEthMiniport::Install(VOID)
{
    INetCfgClass*           pNCClass;
    INetCfg*                pInetCfg = this->pInetCfg;
    INetCfgClassSetup*      pNCClassSetup = NULL;
    INetCfgComponent*       pNCComponentMiniport = NULL;
    HRESULT                 hret;
    LPWSTR*                 pszwRefs = NULL;
    OBO_TOKEN*              pOboToken = NULL;
    DWORD                   setupFlags = 0;
    LPCWSTR                 pszwAnswerFile = NULL;
    LPCWSTR                 pszwAnswerSections = NULL;

    TraceMsg(L"------>CVEthMiniport::Install.\n");

    hret = pInetCfg->QueryNetCfgClass(&GUID_DEVCLASS_NET,
                                      IID_INetCfgClass,
                                      (void**)&pNCClass);

    if (hret != S_OK)
    {
        TraceMsg(L"3HRESULT (%x)\n", hret);
        return hret;
    }

    hret = pNCClass->QueryInterface(IID_INetCfgClassSetup,
                                    (void**)&pNCClassSetup);

    if (hret != S_OK)
    {
        TraceMsg(L"2HRESULT (%x)\n", hret);
        goto Exit;
    }

    hret = pNCClassSetup->Install(c_szMuxMiniport, // Component identifier
                                  pOboToken,       // Obo token for reference count,check
                                  setupFlags,       // installation setup flags, check
                                  0,                // build identifier if any, check
                                  pszwAnswerFile,
                                  pszwAnswerSections,
                                  &pNCComponentMiniport
                                  );
    if (hret != S_OK)
    {
        TraceMsg(L"1HRESULT (%x)\n", hret);
        goto Exit;
    }

    hret = pNCComponentMiniport->GetInstanceGuid(&this->guidMiniport);

    if (hret != S_OK)
    {
        pNCClassSetup->DeInstall(pNCComponentMiniport,
                                 pOboToken,
                                 pszwRefs);
    }

Exit:
    if (pNCComponentMiniport)
        ReleaseObj(pNCComponentMiniport);

    if (pNCClassSetup)
        ReleaseObj(pNCClassSetup);

    if (pNCClass)
        ReleaseObj(pNCClass);

    TraceMsg(L"<------Install HRESULT (%x)\n", hret);
    return hret;
}

HRESULT CVEthMiniport::DeInstall(VOID)
{
    INetCfgClass*           pNCClass;
    INetCfg*                pInetCfg = this->pInetCfg;
    INetCfgClassSetup*      pNCClassSetup;
    INetCfgComponent*       pNCComponentMiniport;
    HRESULT                 hret;
    LPWSTR*                 pmszwRefs = NULL;
    OBO_TOKEN*              pOboToken = NULL;
    TraceMsg(L"------>DeInstall.\n");
    hret = pInetCfg->QueryNetCfgClass(&GUID_DEVCLASS_NET, IID_INetCfgClass,
                                      (PVOID*)&pNCClass);

    if (hret != S_OK)
        return hret;

    hret = pNCClass->QueryInterface(IID_INetCfgClassSetup,
                                    (PVOID*)&pNCClassSetup);

    if (hret != S_OK)
        return hret;

    hret = FindInstance(pInetCfg,
                        this->guidMiniport,
                        &pNCComponentMiniport);

    if (hret != S_OK)
        goto Exit;

    hret = pNCClassSetup->DeInstall(pNCComponentMiniport,
                             pOboToken,
                             pmszwRefs);

Exit:
    if (pNCComponentMiniport)
        ReleaseObj(pNCComponentMiniport);

    if (pNCClassSetup)
        ReleaseObj(pNCClassSetup);

    if (pNCClass)
        ReleaseObj(pNCClass);
    TraceMsg(L"<------DeInstall (HRESULT(%x).\n", hret);
    return hret;
}

HRESULT CVEthMiniport::ApplyRegistryChanges(VOID)
{
    HKEY                    hkeyAdapterGuid;
    WCHAR                   szAdapterGuid[MAX_PATH + 1];
    WCHAR                   szAdapterGuidKey[MAX_PATH + 1];
    WCHAR                   szMiniportGuid[MAX_PATH + 1];
    LPWSTR                  pDevice = NULL;
    LONG                    result = 0;

    TraceMsg(L"----->CVEthMiniport::ApplyRegistryChanges.\n");

    switch (this->applyAction)
    {
        case kActAdd:
        {
            // Check if the adapter exists
            TraceMsg(L"kActAdd\n");
            StringFromGUID2(this->guidAdapter,
                            szAdapterGuid,
                            MAX_PATH + 1);

            StringCchPrintf(szAdapterGuidKey,
                             celems(szAdapterGuidKey),
                             L"%s\\%s",
                             c_szAdapterList,
                             szAdapterGuid);

            szAdapterGuidKey[MAX_PATH] = '\0';
            result = RegCreateKeyEx(HKEY_LOCAL_MACHINE,
                                      szAdapterGuidKey,
                                      0,
                                      NULL,
                                      REG_OPTION_NON_VOLATILE,
                                      KEY_ALL_ACCESS,
                                      NULL,
                                      &hkeyAdapterGuid,
                                      NULL);

            if (result == ERROR_SUCCESS)
            {
                StringFromGUID2(this->guidMiniport,
                                szMiniportGuid,
                                MAX_PATH + 1);

                pDevice = AddDevicePrefix(szMiniportGuid);

                if (pDevice == NULL)
                {
                    result = ERROR_NOT_ENOUGH_MEMORY;
                    break;
                }

                result = RegSetValueEx(hkeyAdapterGuid,
                                       c_szUpperBindings,
                                       0,
                                       REG_SZ,
                                       (LPBYTE)pDevice,
                                       (wcslen(pDevice) + 1) *
                                       sizeof(WCHAR)
                                       );
                if (result != ERROR_SUCCESS)
                {

                    TraceMsg(L"   Failed to save %s at %s\\%s.\n",
                             result,
                             szAdapterGuidKey,
                             c_szUpperBindings);

                }

                RegCloseKey(hkeyAdapterGuid);
            }
            else
            {
                TraceMsg(L"   Failed to open the registry key: %s.\n",
                         szAdapterGuidKey);
            }

            break;
        }
        case kActRemove:
        {
            // Check if the adapter exists
            TraceMsg(L"kActRemove\n");
            StringFromGUID2(this->guidAdapter,
                            szAdapterGuid,
                            MAX_PATH + 1);


            StringCchPrintf(szAdapterGuidKey,
                            celems(szAdapterGuidKey),
                            L"%s\\%s",
                            c_szAdapterList,
                            szAdapterGuid);

            szAdapterGuidKey[MAX_PATH] = '\0';
            result = RegCreateKeyEx(HKEY_LOCAL_MACHINE,
                                    szAdapterGuidKey,
                                    0,
                                    NULL,
                                    REG_OPTION_NON_VOLATILE,
                                    KEY_ALL_ACCESS,
                                    NULL,
                                    &hkeyAdapterGuid,
                                    NULL);

            if (result == ERROR_SUCCESS)
            {
                StringFromGUID2(this->guidMiniport,
                                szMiniportGuid,
                                MAX_PATH + 1);

                pDevice = AddDevicePrefix(szMiniportGuid);

                if (pDevice == NULL)
                {
                    result = ERROR_NOT_ENOUGH_MEMORY;
                    break;
                }

                result = RegDeleteValue(hkeyAdapterGuid,
                                         c_szUpperBindings);
                if (result != ERROR_SUCCESS)
                {

                    TraceMsg(L"   Failed to delete %s at %s\\%s.\n",
                             pDevice,
                             szAdapterGuidKey,
                             c_szUpperBindings);
                }
            }
        }
    }

    if (pDevice)
        free(pDevice);
    TraceMsg(L"<----- CVEthMiniport::ApplyRegistryChanges (HRESULT (%x)).\n", HRESULT_FROM_WIN32(result));
    return HRESULT_FROM_WIN32(result);
}

HRESULT CVEthMiniport::ApplyPnpChanges(INetCfgPnpReconfigCallback* pfnCallback)
{
    TraceMsg(L"CVEthMiniport::ApplyPnpChanges.\n");
    return S_OK;
}

HRESULT CVEthMiniport::SetConfigAction(tConfigAction applyAction_p)
{
    applyAction = applyAction_p;
    return S_OK;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
