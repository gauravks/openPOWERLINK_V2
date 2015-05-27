#include "stdafx.h"
#include "vethMiniport.h"


CVEthMiniport::CVEthMiniport(INetCfg* pInetCfg, GUID* pGuidMiniport, GUID* pGuidAdapter)
{
    TraceMsg(L"---->Constructor CVEthMiniport.\n");

    this->pInetCfg = pInetCfg;
    this->pInetCfg->AddRef();

    CopyMemory(&this->guidAdapter, pGuidAdapter, sizeof(GUID));
    
    if (pGuidMiniport)
        CopyMemory(&this->guidMiniport, pGuidMiniport, sizeof(GUID));
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