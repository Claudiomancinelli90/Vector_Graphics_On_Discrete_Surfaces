#pragma once

#include <realtime/ext/imgui/imgui.h>
#include <realtime/ext/imgui/imgui_internal.h>
#include <yocto/yocto_math.h>

using yocto::vec3f;

struct ImGuiDataTypeInfo
{
    size_t      Size;
    const char* PrintFmt;   // Unused
    const char* ScanFmt;
};

static const ImGuiDataTypeInfo GDataTypeInfo[] =
{
    { sizeof(char),             "%d",   "%d"    },  // ImGuiDataType_S8
    { sizeof(unsigned char),    "%u",   "%u"    },
    { sizeof(short),            "%d",   "%d"    },  // ImGuiDataType_S16
    { sizeof(unsigned short),   "%u",   "%u"    },
    { sizeof(int),              "%d",   "%d"    },  // ImGuiDataType_S32
    { sizeof(unsigned int),     "%u",   "%u"    },
#ifdef _MSC_VER
    { sizeof(ImS64),            "%I64d","%I64d" },  // ImGuiDataType_S64
    { sizeof(ImU64),            "%I64u","%I64u" },
#else
    { sizeof(ImS64),            "%lld", "%lld"  },  // ImGuiDataType_S64
    { sizeof(ImU64),            "%llu", "%llu"  },
#endif
    { sizeof(float),            "%f",   "%f"    },  // ImGuiDataType_Float (float are promoted to double in va_arg)
    { sizeof(double),           "%f",   "%lf"   },  // ImGuiDataType_Double
};
IM_STATIC_ASSERT(IM_ARRAYSIZE(GDataTypeInfo) == ImGuiDataType_COUNT);

namespace ImGui {
  IMGUI_API bool InputVec3f(const char* label, vec3f& v, const char* format[3] = NULL, ImGuiInputTextFlags flags = 0);
  IMGUI_API bool InputScalarN(const char* label, ImGuiDataType data_type, void* v, int components, const void* step, const void* step_fast, const char* format[3], ImGuiInputTextFlags flags);
}