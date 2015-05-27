// notify.h : Declaration of the CNotify

#pragma once
#include "resource.h"       // main symbols

#include <netcfgn.h>
#include "common.h"
#include "phyAdapter.h"
#include <notifyObj_i.h>
#include <list>

#if defined(_WIN32_WCE) && !defined(_CE_DCOM) && !defined(_CE_ALLOW_SINGLE_THREADED_OBJECTS_IN_MTA)
#error "Single-threaded COM objects are not properly supported on Windows CE platform, such as the Windows Mobile platforms that do not include full DCOM support. Define _CE_ALLOW_SINGLE_THREADED_OBJECTS_IN_MTA to force ATL to support creating single-thread COM object's and allow use of it's single-threaded COM object implementations. The threading model in your rgs file was set to 'Free' as that is the only threading model supported in non DCOM Windows CE platforms."
#endif

using namespace ATL;


// CNotify

class ATL_NO_VTABLE CNotify :
    public CComObjectRootEx<CComSingleThreadModel>,
    public CComCoClass<CNotify, &CLSID_CNotify>,
    public ISupportErrorInfo,
    public IDispatchImpl<INotify, &IID_INotify, &LIBID_notifyObjLib, /*wMajor =*/ 1, /*wMinor =*/ 0>,
    public INetCfgComponentControl,
    public INetCfgComponentSetup,
    public INetCfgComponentPropertyUi,
    public INetCfgComponentNotifyBinding,
    public INetCfgComponentNotifyGlobal
{
public:
    // Constructor
    CNotify();
    // Destructor
    ~CNotify();

    BEGIN_COM_MAP(CNotify)
        COM_INTERFACE_ENTRY(INotify)
        COM_INTERFACE_ENTRY(IDispatch)
        COM_INTERFACE_ENTRY(ISupportErrorInfo)
        COM_INTERFACE_ENTRY(INetCfgComponentControl)
        COM_INTERFACE_ENTRY(INetCfgComponentSetup)
        COM_INTERFACE_ENTRY(INetCfgComponentPropertyUi)
        COM_INTERFACE_ENTRY(INetCfgComponentNotifyBinding)
        COM_INTERFACE_ENTRY(INetCfgComponentNotifyGlobal)
    END_COM_MAP()

    DECLARE_REGISTRY_RESOURCEID(IDR_NOTIFY)
    // ISupportsErrorInfo
    STDMETHOD(InterfaceSupportsErrorInfo)(REFIID riid);

    // INetCfgComponentControl
    STDMETHOD(Initialize)(IN INetCfgComponent* pIComp_p, IN INetCfg* pINetCfg_p,
                          IN BOOL fInstalling_p);
    STDMETHOD(CancelChanges)();
    STDMETHOD(ApplyRegistryChanges)();
    STDMETHOD(ApplyPnpChanges)(IN INetCfgPnpReconfigCallback* pICallback);

    // INetCfgComponentSetup
    STDMETHOD(Install)(IN DWORD setupFlags);
    STDMETHOD(Upgrade)(IN DWORD setupFlags, IN DWORD upgradeFromBuildNo);
    STDMETHOD(ReadAnswerFile)(IN PCWSTR pszAnswerFile, IN PCWSTR pszAnswerSection);
    STDMETHOD(Removing)();

    // INetCfgComponentPropertyUi
    STDMETHOD(QueryPropertyUi)(IN IUnknown* pUnk);
    STDMETHOD(SetContext)(IN IUnknown* pUnk);
    STDMETHOD(MergePropPages)(IN OUT DWORD* pDefPages,
                              OUT LPBYTE* pahpspPrivate,
                              OUT UINT* pcPrivate,
                              IN HWND hwndParent,
                              OUT PCWSTR* pszStartPage);
    STDMETHOD(ValidateProperties)(HWND hwndSheet);
    STDMETHOD(CancelProperties)();
    STDMETHOD(ApplyProperties)();

    // INetCfgNotifyBinding
    STDMETHOD(QueryBindingPath)(IN DWORD changeFlag, IN INetCfgBindingPath* pNetCfgBindPath);
    STDMETHOD(NotifyBindingPath)(IN DWORD changeFlag, IN INetCfgBindingPath* pNetCfgBindPath);

    // INetCfgNotifyGlobal
    STDMETHOD(GetSupportedNotifications)(OUT DWORD* pNotificationFlag);
    STDMETHOD(SysQueryBindingPath)(IN DWORD changeFlag, IN INetCfgBindingPath* pNetCfgBindPath);
    STDMETHOD(SysNotifyBindingPath)(IN DWORD changeFlag, IN INetCfgBindingPath* pNetCfgBindPath);
    STDMETHOD(SysNotifyComponent)(IN DWORD changeFlag, IN INetCfgComponent* pNetCfgComponent);

    DECLARE_PROTECT_FINAL_CONSTRUCT()

    HRESULT FinalConstruct()
    {
        return S_OK;
    }

    void FinalRelease()
    {
    }

private:
    //
    // Private member variables.
    //

    INetCfgComponent*    pInetComponent;  // Protocol's Net Config component
    INetCfg*             pInetCfg;
    tConfigAction        applyAction;

    std::list<CPhyAdapter*> pPhyAdapterList;

    HRESULT initalizeAdapters(VOID);
    HRESULT getUpperAndLowerBindings(INetCfgBindingPath* pInetBindPath_p,
                                     INetCfgComponent** ppUpperComponent_p,
                                     INetCfgComponent** ppLowerComponent_p);
    HRESULT addAdapter(INetCfgComponent* pAdapter_p);
    HRESULT removeAdapter(INetCfgComponent* pAdapter_p);
    HRESULT addMiniport(CPhyAdapter* pPhyMiniport_p, GUID* pAdapterGuid_p,
                        INetCfgComponent* pInetComponent_p);
    HRESULT removeMiniport(CPhyAdapter* pVEthMiniport_p, GUID* pAdapterGuid_p);

    void enableProtocolBindings(INetCfgComponent* pAdapter_p, BOOL fEnable_p);
    BOOL checkBindingStatus(INetCfgBindingPath *pBindPath_p);
    HRESULT findAdapter(GUID* pGuidAdapter_p, CPhyAdapter** pPhyAdapter_p);


};

OBJECT_ENTRY_AUTO(__uuidof(CNotify), CNotify)
