// dllmain.h : Declaration of module class.

class CnotifyObjModule : public ATL::CAtlDllModuleT < CnotifyObjModule >
{
public:
    DECLARE_LIBID(LIBID_notifyObjLib)
    DECLARE_REGISTRY_APPID_RESOURCEID(IDR_NOTIFYOBJ, "{67AA49B1-4042-4959-847D-27C13A2DBE8E}")
};

extern class CnotifyObjModule _AtlModule;
