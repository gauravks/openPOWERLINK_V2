// notify.cpp : Implementation of CNotify

#include "stdafx.h"
#include "notify.h"


// CNotify

CNotify::CNotify(VOID) : pInetComponent(NULL),
                         pInetCfg(NULL),
                         applyAction(kActUnknown),
                         pPhyAdapterList(std::list<CPhyAdapter*>())
{
    TraceMsg(L"CNotify.\n");
}

CNotify::~CNotify(VOID)
{
    CPhyAdapter*        pPhyAdapter = NULL;
    GUID                adaptGuid;
    WCHAR               szAdaptGuid[MAX_PATH + 1];;

    TraceMsg(L"----->~CNotify.\n");

    ReleaseObj(pInetComponent);
    ReleaseObj(pInetCfg);

    TraceMsg(L"<-----~pPhyAdapterList.size %d", pPhyAdapterList.size());
    while (pPhyAdapterList.size() > 0)
    {
        pPhyAdapter = pPhyAdapterList.front();
        if (pPhyAdapter != NULL)
            delete pPhyAdapter;

        pPhyAdapterList.pop_front();
    }

    TraceMsg(L"<-----~CNotify.\n");
}

STDMETHODIMP CNotify::InterfaceSupportsErrorInfo(REFIID riid)
{
    static const IID* const arr[] =
    {
        &IID_INotify
    };
    TraceMsg(L"InterfaceSupportsErrorInfo.\n");
    for (int i = 0; i < sizeof(arr) / sizeof(arr[0]); i++)
    {
        if (InlineIsEqualGUID(*arr[i], riid))
            return S_OK;
    }
    return S_FALSE;
}

// INetCfgComponentControl
STDMETHODIMP CNotify::Initialize(IN INetCfgComponent* pIComp_p, IN INetCfg* pINetCfg_p,
                                 IN BOOL fInstalling_p)
{
    HRESULT		hret = S_OK;
    TraceMsg(L"Initialize.\n");
    pInetComponent = pIComp_p;
    pInetCfg = pINetCfg_p;

    if (pInetComponent)
    {
        TraceMsg(L"Add Ref pInetComponent\n");
        pInetComponent->AddRef();
    }

    if (pInetCfg)
    {
        TraceMsg(L"Add Ref pInetCfg\n");
        pInetCfg->AddRef();
    }

    // We initialize the adapters and virtual miniports if this is not a installation

    if (!fInstalling_p)
        hret = initalizeAdapters();

    TraceMsg(L"(HRESULT (%x).\n", hret);

    return hret;
}

STDMETHODIMP CNotify::CancelChanges()
{
    // Nothing to be done
    TraceMsg(L"CancelChanges.\n");
    return S_OK;
}

STDMETHODIMP CNotify::ApplyRegistryChanges()
{
    CPhyAdapter*    pPhyAdapter = NULL;
    HRESULT         hret = S_OK;
    std::list<CPhyAdapter*>::iterator iterator = pPhyAdapterList.begin();
    TraceMsg(L"---->ApplyRegistryChanges.\n");

    for (; iterator != pPhyAdapterList.end(); iterator++)
    {
        pPhyAdapter = *iterator;
        hret = pPhyAdapter->ApplyRegistryChanges();
    }

    TraceMsg(L"<----ApplyRegistryChanges.\n");
    return hret;
}

STDMETHODIMP CNotify::ApplyPnpChanges(IN INetCfgPnpReconfigCallback* pICallback)
{
    CPhyAdapter*    pPhyAdapter = NULL;
    HRESULT         hret = S_OK;

    std::list<CPhyAdapter*>::iterator iterator = pPhyAdapterList.begin();

    TraceMsg(L"----->ApplyPnpChanges.\n");

    for (; iterator != pPhyAdapterList.end(); iterator++)
    {
        pPhyAdapter = *iterator;
        hret = pPhyAdapter->ApplyPnpChanges(pICallback);
    }
    TraceMsg(L"<-----ApplyPnpChanges.\n");
    return hret;
}

// INetCfgComponentSetup
STDMETHODIMP CNotify::Install(IN DWORD setupFlags)
{
    UNREFERENCED_PARAMETER(setupFlags);

    applyAction = kActInstall;

    TraceMsg(L"Install.\n");
    return S_OK;
}

STDMETHODIMP CNotify::Upgrade(IN DWORD setupFlags, IN DWORD upgradeFromBuildNo)
{
    UNREFERENCED_PARAMETER(setupFlags);
    UNREFERENCED_PARAMETER(upgradeFromBuildNo);

    TraceMsg(L"   setupFlags = %x, upgradeFromBuildNo = %x\n",
             setupFlags,
             upgradeFromBuildNo);

    TraceMsg(L"Upgrade.\n");
    return S_OK;
}

STDMETHODIMP CNotify::ReadAnswerFile(IN PCWSTR pszAnswerFile, IN PCWSTR pszAnswerSection)
{
    UNREFERENCED_PARAMETER(pszAnswerFile);
    UNREFERENCED_PARAMETER(pszAnswerSection);

    TraceMsg(L"ReadAnswerFile.\n");
    return S_OK;
}

STDMETHODIMP CNotify::Removing()
{
    TraceMsg(L"Removing.\n");
    return S_OK;
}

// INetCfgNotifyBinding
STDMETHODIMP CNotify::QueryBindingPath(IN DWORD changeFlag, IN INetCfgBindingPath* pNetCfgBindPath)
{
    UNREFERENCED_PARAMETER(changeFlag);
    UNREFERENCED_PARAMETER(pNetCfgBindPath);
    TraceMsg(L"QueryBindingPath.\n");
    DumpChangeFlag(changeFlag);
    DumpBindingPath(pNetCfgBindPath);

    return S_OK;
}

STDMETHODIMP CNotify::NotifyBindingPath(IN DWORD changeFlag, IN INetCfgBindingPath* pNetCfgBindPath)
{
    INetCfgComponent*    pLowerComponent = NULL;
    INetCfgComponent*    pUpperComponent = NULL;
    LPWSTR               pszwInfIdLower = NULL;
    LPWSTR               pszwInfIdUpper = NULL;
    DWORD                characteristics;
    HRESULT              hret = S_OK;

    TraceMsg(L"----->NotifyBindingPath.\n");
    DumpChangeFlag(changeFlag);
    DumpBindingPath(pNetCfgBindPath);
    // Copied from sample:
    // We are only interested to know 1) when a component is installed
    // and we are binding to it i.e. dwChangeFlag = NCN_ADD | NCN_ENABLE
    // and 2) when a component is removed to which we are bound i.e.
    // dwChangeFlag = NCN_REMOVE | NCN_ENABLE. dwChangeFlag is never
    // set to NCN_ADD or NCN_REMOVE only. So, checking for NCN_ENABLE
    // covers the case of NCN_ADD | NCN_ENABLE and checking for NCN_REMOVE
    // covers the case of NCN_REMOVE | NCN_ENABLE. We don't care about
    // NCN_ADD | NCN_DISABLE (case 1) and NCN_REMOVE | NCN_DISABLE (case 2).

    if (changeFlag & (NCN_ENABLE | NCN_REMOVE))
    {
        // get the upper and lower bindings
        hret = getUpperAndLowerBindings(pNetCfgBindPath, &pUpperComponent, &pLowerComponent);
        if (hret != S_OK)
            return hret;

        hret = pLowerComponent->GetCharacteristics(&characteristics);
        if (hret != S_OK)
            goto Exit;

        hret = pUpperComponent->GetId(&pszwInfIdUpper);
        if (hret != S_OK)
            goto Exit;

        hret = pLowerComponent->GetId(&pszwInfIdLower);
        if (hret != S_OK)
        {
            if (pszwInfIdUpper)
                CoTaskMemFree(pszwInfIdUpper);
            goto Exit;
        }

        // Bind only to a physical device

        if (characteristics & NCF_VIRTUAL)
        {
            goto ExitFree;
        }
        else
        {
            if (characteristics & NCF_PHYSICAL)
            {
                if (_wcsicmp(pszwInfIdUpper, c_szMuxProtocol))
                     goto ExitFree;

                // Upper binding in our driver, add the adapter
                if (changeFlag & NCN_ADD)
                {
                    hret = addAdapter(pLowerComponent);
                    if (hret != S_OK)
                    {
                        TraceMsg(L"Unable to Add adapter error %x", hret);
                        goto ExitFree;
                    }

                    applyAction = kActAdd;
                }
                else
                {
                    if (changeFlag & NCN_REMOVE)
                    {
                        hret = removeAdapter(pLowerComponent);
                        if (hret != S_OK)
                        {
                            TraceMsg(L"Unable to Remove adapter error %x", hret);
                            goto ExitFree;
                        }
                        applyAction = kActRemove;
                    }
                }
            }
        }
    }

ExitFree:
    if (pszwInfIdUpper)
        CoTaskMemFree(pszwInfIdUpper);
    if (pszwInfIdLower)
        CoTaskMemFree(pszwInfIdLower);
Exit:
    ReleaseObj(pLowerComponent);
    ReleaseObj(pUpperComponent);

    TraceMsg(L"<-----NotifyBindingPath HRESULT(%x).\n", hret);
    return hret;
}

// INetCfgNotifyGlobal
STDMETHODIMP CNotify::GetSupportedNotifications(OUT DWORD* pNotificationFlag)
{
    TraceMsg(L"GetSupportedNotifications.\n");
    *pNotificationFlag = NCN_NET | NCN_NETTRANS | NCN_ADD | NCN_REMOVE |
                         NCN_BINDING_PATH | NCN_ENABLE | NCN_DISABLE;
    return S_OK;
}

STDMETHODIMP CNotify::SysQueryBindingPath(IN DWORD changeFlag, IN INetCfgBindingPath* pNetCfgBindPath)
{
    INetCfgComponent*    pLowerComponent = NULL;
    INetCfgComponent*    pUpperComponent = NULL;
    LPWSTR               pszwInfIdLower = NULL;
    LPWSTR               pszwInfIdUpper = NULL;
    DWORD                characteristics;
    HRESULT              hret = S_OK;

    TraceMsg(L"---->SysQueryBindingPath\n");
    DumpChangeFlag(changeFlag);
    DumpBindingPath(pNetCfgBindPath);

    if (changeFlag & NCN_ENABLE)
    {
        // get the upper and lower bindings
        hret = getUpperAndLowerBindings(pNetCfgBindPath, &pUpperComponent, &pLowerComponent);
        if (hret != S_OK)
            return hret;

        hret = pLowerComponent->GetCharacteristics(&characteristics);
        if (hret != S_OK)
            goto Exit;

        hret = pUpperComponent->GetId(&pszwInfIdUpper);
        if (hret != S_OK)
            goto Exit;

        hret = pLowerComponent->GetId(&pszwInfIdLower);
        if (hret != S_OK)
        {
            if (pszwInfIdUpper)
                CoTaskMemFree(pszwInfIdUpper);
            goto Exit;
        }

        // Enable binding only physical bindings
        if (characteristics & NCF_VIRTUAL)
        {
            if (!_wcsicmp(pszwInfIdLower, c_szMuxMiniport) &&
                !_wcsicmp(pszwInfIdUpper, c_szMuxProtocol))
            {
                hret = NETCFG_S_DISABLE_QUERY;
                goto ExitFree;
            }
        }
        else
        {
            if (characteristics & NCF_PHYSICAL)
            {
                // Enable binding only if upper protocol is our driver
                if (_wcsicmp(pszwInfIdUpper, c_szMuxProtocol))
                {
                    hret = NETCFG_S_DISABLE_QUERY;
                }
            }
        }
    }

ExitFree:
    if (pszwInfIdUpper)
        CoTaskMemFree(pszwInfIdUpper);
    if (pszwInfIdLower)
        CoTaskMemFree(pszwInfIdLower);
Exit:
    ReleaseObj(pLowerComponent);
    ReleaseObj(pUpperComponent);

    TraceMsg(L"<----SysQueryBindingPath (HRESULT (%x).\n", hret);
    return hret;
}

STDMETHODIMP CNotify::SysNotifyBindingPath(IN DWORD changeFlag, IN INetCfgBindingPath* pNetCfgBindPath)
{
    UNREFERENCED_PARAMETER(changeFlag);
    UNREFERENCED_PARAMETER(pNetCfgBindPath);

    TraceMsg(L"---->SysNotifyBindingPath.\n");
    DumpChangeFlag(changeFlag);
    DumpBindingPath(pNetCfgBindPath);
    TraceMsg(L"<----SysNotifyBindingPath.\n");
    return S_OK;
}

STDMETHODIMP CNotify::SysNotifyComponent(IN DWORD changeFlag, IN INetCfgComponent* pNetCfgComponent)
{
    UNREFERENCED_PARAMETER(changeFlag);
    UNREFERENCED_PARAMETER(pNetCfgComponent);
    TraceMsg(L"---->SysNotifyComponent.\n");
    DumpChangeFlag(changeFlag);
    DumpComponent(pNetCfgComponent);
    TraceMsg(L"<----SysNotifyComponent.\n");
    return S_OK;
}

STDMETHODIMP CNotify::MergePropPages(IN OUT DWORD* pDefPages,
                                     OUT LPBYTE* pahpspPrivate,
                                     OUT UINT* pcPrivate,
                                     IN HWND hwndParent,
                                     OUT PCWSTR* pszStartPage)
{
    // We don't generate any property pages.
    UNREFERENCED_PARAMETER(pDefPages);
    UNREFERENCED_PARAMETER(hwndParent);

    *pcPrivate = 0;
    *pahpspPrivate = NULL;
    *pszStartPage = NULL;

    TraceMsg(L"MergePropPages.\n");
    return S_OK;
}

STDMETHODIMP CNotify::ValidateProperties(HWND hwndSheet)
{
    UNREFERENCED_PARAMETER(hwndSheet);
    TraceMsg(L"ValidateProperties.\n");
    return S_OK;
}

STDMETHODIMP CNotify::CancelProperties()
{
    TraceMsg(L"CancelProperties.\n");
    return S_OK;
}

STDMETHODIMP CNotify::ApplyProperties()
{
    TraceMsg(L"ApplyProperties.\n");
    return S_OK;
}


// INetCfgComponentPropertyUi
STDMETHODIMP CNotify::QueryPropertyUi(IN IUnknown* pUnk)
{
    TraceMsg(L"QueryPropertyUi.\n");
    return S_OK;
}

STDMETHODIMP CNotify::SetContext(IN IUnknown* pUnk)
{
    TraceMsg(L"SetContext.\n");
    return S_OK;
}
HRESULT CNotify::initalizeAdapters(VOID)
{
    HKEY                 hkeyAdapterList;
    WCHAR                szAdapterGuid[MAX_PATH + 1];
    DWORD                adapterDispositon; // Identifies if the key was created or read
    GUID                 adapterGuid;
    DWORD                index;
    LONG                 lResult = ERROR_SUCCESS;
    CPhyAdapter*         pAdapter;
    TraceMsg(L"----->initalizeAdapters.\n");
    lResult = RegCreateKeyEx(HKEY_LOCAL_MACHINE, // Root key for the new entry
                             c_szAdapterList,    // Sub key
                             0,                  // reserved
                             NULL,               // Class
                             REG_OPTION_NON_VOLATILE, // Non-volatile entry to preserved during power cycles
                             KEY_ALL_ACCESS,        // Standard acces parameters
                             NULL,                  // no security structure
                             &hkeyAdapterList,      // returned key
                             &adapterDispositon     // adapter disposition
                             );

    if (lResult != ERROR_SUCCESS)
    {
        return HRESULT_FROM_WIN32(lResult);
    }

    // If a new key is created then there are no adapters to bind to.
    if (adapterDispositon == REG_CREATED_NEW_KEY)
    {
        RegCloseKey(hkeyAdapterList);
        return HRESULT_FROM_WIN32(lResult);
    }

    // If key existed, we read the adapter list and initialize miniports.
    // TODO: add a check here to restrict the target Adapter.
    lResult = RegEnumKey(hkeyAdapterList,
                          0,
                          szAdapterGuid,
                          MAX_PATH + 1);

    for (index = 1; lResult == ERROR_SUCCESS; ++index)
    {
        // read the adapter GUID
        szAdapterGuid[MAX_PATH] = '\0';

        CLSIDFromString(szAdapterGuid,
                        &adapterGuid);
        TraceMsg(L"AddAdapter %s.\n", szAdapterGuid);
        pAdapter = new CPhyAdapter(pInetCfg, adapterGuid);

        if (pAdapter == NULL)
        {
            lResult = ERROR_NOT_ENOUGH_MEMORY;
            goto Exit;
        }

        pAdapter->Initialize();
        pAdapter->SetConfigAction(kActUpdate);

        pPhyAdapterList.push_back(pAdapter);

        lResult = RegEnumKey(hkeyAdapterList,
                              index,
                              szAdapterGuid,
                              MAX_PATH + 1);
        lResult = ERROR_INVALID_FUNCTION;
    }

    lResult = ERROR_SUCCESS;

Exit:
    RegCloseKey(hkeyAdapterList);
    TraceMsg(L"<-----initalizeAdapters. HRESULT(%x)\n", HRESULT_FROM_WIN32(lResult));

    return HRESULT_FROM_WIN32(lResult);
}

HRESULT CNotify::getUpperAndLowerBindings(INetCfgBindingPath* pInetBindPath_p,
                                          INetCfgComponent** ppUpperComponent_p,
                                          INetCfgComponent** ppLowerComponent_p)
{
    IEnumNetCfgBindingInterface*    pEnumNetCfg;
    INetCfgBindingInterface*        pInetBindInterface;
    ULONG                           count;
    HRESULT                         hret;
    TraceMsg(L"------>getUpperAndLowerBindings.\n");
    *ppUpperComponent_p = NULL;
    *ppLowerComponent_p = NULL;

    hret = pInetBindPath_p->EnumBindingInterfaces(&pEnumNetCfg);

    if (hret != S_OK)
        return hret;

    hret = pEnumNetCfg->Next(1, &pInetBindInterface, &count);

    if (hret != S_OK)
    {
        ReleaseObj(pEnumNetCfg);
        return hret;
    }

    hret = pInetBindInterface->GetLowerComponent(ppLowerComponent_p);

    if (hret != S_OK)
    {
        ReleaseObj(pInetBindInterface);
        ReleaseObj(pEnumNetCfg);
        if (ppLowerComponent_p != NULL)
            ReleaseObj(*ppLowerComponent_p);

        return hret;
    }

    hret = pInetBindInterface->GetUpperComponent(ppUpperComponent_p);

    ReleaseObj(pInetBindInterface);
    ReleaseObj(pEnumNetCfg);

    TraceMsg(L"<------getUpperAndLowerBindings HRESULT(%x)\n",hret);
    return hret;
}

HRESULT CNotify::addAdapter(INetCfgComponent* pAdapter_p)
{
    GUID                     guidAdapter;
    CPhyAdapter              *pAdapter;
    HRESULT                  hret = S_OK;
    TraceMsg(L"----->AddAdapter \n");

    hret = pAdapter_p->GetInstanceGuid(&guidAdapter);
    // TODO: add a check here to restrict the target Adapter.
    if (hret != S_OK)
        return hret;

    pAdapter = new CPhyAdapter(this->pInetCfg, guidAdapter);

    if (pAdapter == NULL)
        return HRESULT_FROM_WIN32(ERROR_NOT_ENOUGH_MEMORY);

    hret = addMiniport(pAdapter, &guidAdapter, pAdapter_p);

    if (hret != S_OK)
    {
        delete pAdapter;
        TraceMsg(L"<-------AddAdapter HRESULT %x\n", hret);
        return hret;
    }

    pAdapter->SetConfigAction(kActAdd);

    pPhyAdapterList.push_back(pAdapter);

    TraceMsg(L"<-------AddAdapter HRESULT %x\n", hret);
    return hret;
}

HRESULT CNotify::removeAdapter(INetCfgComponent* pAdapter_p)
{
    GUID                     guidAdapter;
    CPhyAdapter*             pAdapter = NULL;
    HRESULT                  hret = S_OK;
    
    TraceMsg(L"----->removeAdapter.\n");
    hret = pAdapter_p->GetInstanceGuid(&guidAdapter);

    if (hret != S_OK)
        return hret;

    hret = findAdapter(&guidAdapter, &pAdapter);

    if (hret != S_OK || pAdapter == NULL)
    {
        TraceMsg(L"Adapter not Found %x\n", hret);
        return hret;
    }

    pAdapter->SetConfigAction(kActRemove);
    pAdapter->RemoveMiniport();

    enableProtocolBindings(pAdapter_p, TRUE);

    //pPhyAdapterList.remove(pAdapter);

    //delete pAdapter;

    TraceMsg(L"<-----removeAdapter.\n");
    return hret;
}

HRESULT CNotify::addMiniport(CPhyAdapter* pPhyAdapter_p, GUID* pAdapterGuid_p,
                             INetCfgComponent* pInetComponent_p)
{
    CVEthMiniport*          pMiniport;
    INetCfgComponent*       pInetAdapter;
    HRESULT                 hret = S_OK;
    WCHAR                   szGuidString[MAX_PATH + 1];
    TraceMsg(L"----->addMiniport.\n");
    if (pPhyAdapter_p->MiniportPresent())
        return HRESULT_FROM_WIN32(ERROR_NO_SYSTEM_RESOURCES);

    pMiniport = new CVEthMiniport(pInetCfg, NULL, pAdapterGuid_p);

    if (pMiniport == NULL)
        return HRESULT_FROM_WIN32(ERROR_NOT_ENOUGH_MEMORY);

    hret = pMiniport->Install();
    if (hret != S_OK)
    {
        delete pMiniport;
        return hret;
    }

    hret = pPhyAdapter_p->AddMiniport(pMiniport);
    if (hret != S_OK)
    {
        pMiniport->DeInstall();
        delete pMiniport;
        return hret;
    }

    // Disable protocol bindings on the adapter.
    enableProtocolBindings(pInetComponent_p, FALSE);

    StringFromGUID2(*pAdapterGuid_p, szGuidString, MAX_PATH + 1);
    TraceMsg(L"<-----addMiniport %s HRESULT %x.\n", szGuidString, hret);

    return hret;
}

HRESULT CNotify::removeMiniport(CPhyAdapter* pPhyAdapter_p, GUID* pAdapterGuid_p)
{
    WCHAR                   szGuidString[MAX_PATH + 1];
    HRESULT                 hret;
    INetCfgComponent*       pIComponent = NULL;

    hret = pPhyAdapter_p->RemoveMiniport();

    if (hret != S_OK)
        return hret;

    // enable the protocol bindings on the adapter
    hret = FindInstance(pInetCfg, *pAdapterGuid_p, &pIComponent);

    if (hret != S_OK)
        return hret;

    enableProtocolBindings(pIComponent, TRUE);
    ReleaseObj(pIComponent);

    StringFromGUID2(*pAdapterGuid_p, szGuidString, MAX_PATH + 1);
    TraceMsg(L"removeMiniport %s.\n", szGuidString);
    return S_OK;
}

void CNotify::enableProtocolBindings(INetCfgComponent* pAdapter_p, BOOL fEnable_p)
{
    IEnumNetCfgBindingPath*       pEnumNetCfgPath = NULL;
    INetCfgComponentBindings*     pNetCfgComponentBind = NULL;
    INetCfgBindingPath*           pNetCfgBindPath;
    ULONG                         retCount;
    HRESULT                       hret;
    TraceMsg(L"---->enableProtocolBindings ");

    if (fEnable_p)
        TraceMsg(L"....ENABLE\n");
    else
        TraceMsg(L"....DISABLE\n");

    hret = pAdapter_p->QueryInterface(IID_INetCfgComponentBindings,
                                      (PVOID *)&pNetCfgComponentBind);
    if (hret != S_OK)
        return;

    hret = pNetCfgComponentBind->EnumBindingPaths(EBP_ABOVE,
                                                  &pEnumNetCfgPath);

    if (hret != S_OK)
        return;

    ReleaseObj(pNetCfgComponentBind);

    hret = pEnumNetCfgPath->Next(1, &pNetCfgBindPath, &retCount);

    while (hret == S_OK)
    {
        if (!checkBindingStatus(pNetCfgBindPath))
        {
            pNetCfgBindPath->Enable(fEnable_p);
        }

        ReleaseObj(pNetCfgBindPath);
        pNetCfgBindPath = NULL;
        hret = pEnumNetCfgPath->Next(1, &pNetCfgBindPath, &retCount);
    }

    ReleaseObj(pEnumNetCfgPath);

    TraceMsg(L"<----enableProtocolBindings.\n");

}

BOOL CNotify::checkBindingStatus(INetCfgBindingPath *pBindPath_p)
{
    IEnumNetCfgBindingInterface* pEnumNCBInterface = NULL;
    INetCfgBindingInterface*     pNCBInterface = NULL;
    INetCfgComponent*            pNCComponentUpper;
    LPWSTR                       lpszIdUpper;
    HRESULT                      hret = S_OK;
    BOOL                         bExist = FALSE;
    ULONG                        retCount;

    hret = pBindPath_p->EnumBindingInterfaces(&pEnumNCBInterface);

    if (hret != S_OK)
    {
        TraceMsg(L"   Couldn't get the binding interface enumerator.\n");
        return bExist;
    }

    hret = pEnumNCBInterface->Next(1, &pNCBInterface, &retCount);

    while (!bExist && (hret == S_OK))
    {
        // check if the upper component is openPOWERLINK
        hret = pNCBInterface->GetUpperComponent(&pNCComponentUpper);
        TraceMsg(L"1hret = %x\n", hret);
        if (hret == S_OK)
        {
            hret = pNCComponentUpper->GetId(&lpszIdUpper);
            TraceMsg(L"2hret = %x\n", hret);
            if (hret == S_OK)
            {
                bExist = !_wcsicmp(lpszIdUpper, c_szMuxProtocol);
                CoTaskMemFree(lpszIdUpper);
            }

            ReleaseObj(pNCComponentUpper);
        }

        ReleaseObj(pNCBInterface);

        if (!bExist)
        {
            pNCBInterface = NULL;
            hret = pEnumNCBInterface->Next(1, &pNCBInterface, &retCount);
            TraceMsg(L"3hret = %x\n", hret);
        }
    }

    ReleaseObj(pEnumNCBInterface);

    TraceMsg(L"checkBindingStatus %x.\n", bExist);
    return bExist;
}

HRESULT CNotify::findAdapter(GUID* pGuidAdapter_p, CPhyAdapter** pPhyAdapter_p)
{
    CPhyAdapter* pPhyAdapt = NULL;
    GUID         adaptGuid;
    WCHAR        szKeyGuid[MAX_PATH + 1];
    WCHAR        szAdaptGuid[MAX_PATH + 1];

    StringFromGUID2(*pGuidAdapter_p, szKeyGuid, MAX_PATH + 1);
    TraceMsg(L"findAdapter %s.\n", szKeyGuid);
    for (std::list<CPhyAdapter*>::iterator iterator = pPhyAdapterList.begin();
         iterator != pPhyAdapterList.end(); iterator++)
    {
        pPhyAdapt = *iterator;

        if (pPhyAdapt != NULL)
        {
            pPhyAdapt->GetAdapterGUID(&adaptGuid);
            StringFromGUID2(adaptGuid, szAdaptGuid, MAX_PATH + 1);
            TraceMsg(L"TestAdapter %s.\n", szAdaptGuid);
            if (IsEqualGUID(adaptGuid, *pGuidAdapter_p))
            {
                *pPhyAdapter_p = pPhyAdapt;
                return S_OK;
            }
        }
    }
    TraceMsg(L"....Not found .\n");
    return HRESULT_FROM_WIN32(ERROR_NOT_FOUND);
}