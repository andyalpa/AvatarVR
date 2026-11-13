// Read-only current-adapter scheduling query. No game attach, registry writes,
// driver policy changes, rendering work or OpenXR session.
#include <windows.h>
#include <d3dkmthk.h>
#include <dxgi1_2.h>
#include <cstdio>
#include <cwchar>

int main() {
  IDXGIFactory1* factory = nullptr;
  HRESULT hr = CreateDXGIFactory1(IID_PPV_ARGS(&factory));
  if (FAILED(hr)) return 1;
  unsigned queried = 0;
  for (UINT i = 0; ; ++i) {
    IDXGIAdapter1* adapter = nullptr;
    if (factory->EnumAdapters1(i, &adapter) == DXGI_ERROR_NOT_FOUND) break;
    if (!adapter) continue;
    DXGI_ADAPTER_DESC1 desc = {};
    hr = adapter->GetDesc1(&desc);
    adapter->Release();
    if (FAILED(hr) || (desc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE)) continue;
    D3DKMT_OPENADAPTERFROMLUID opened = {};
    opened.AdapterLuid = desc.AdapterLuid;
    const NTSTATUS open_status = D3DKMTOpenAdapterFromLuid(&opened);
    std::wprintf(L"adapter %u: %ls, vendor 0x%04X, LUID %08lX:%08lX\n", i, desc.Description,
                 desc.VendorId, static_cast<unsigned long>(desc.AdapterLuid.HighPart), desc.AdapterLuid.LowPart);
    if (open_status < 0) {
      std::wprintf(L"  OpenAdapter failed: 0x%08lX\n", static_cast<unsigned long>(open_status));
      continue;
    }
    D3DKMT_WDDM_2_7_CAPS caps = {};
    D3DKMT_QUERYADAPTERINFO query = {};
    query.hAdapter = opened.hAdapter;
    query.Type = KMTQAITYPE_WDDM_2_7_CAPS;
    query.pPrivateDriverData = &caps;
    query.PrivateDriverDataSize = sizeof(caps);
    const NTSTATUS status = D3DKMTQueryAdapterInfo(&query);
    if (status >= 0) {
      ++queried;
      std::wprintf(L"  hardware scheduling: supported=%u enabled=%u default=%u (current WDDM adapter state)\n",
                  caps.HwSchSupported, caps.HwSchEnabled, caps.HwSchEnabledByDefault);
    } else std::wprintf(L"  WDDM 2.7 caps query failed: 0x%08lX\n", static_cast<unsigned long>(status));
    D3DKMT_CLOSEADAPTER close = {};
    close.hAdapter = opened.hAdapter;
    D3DKMTCloseAdapter(&close);
  }
  factory->Release();
  return queried ? 0 : 2;
}
