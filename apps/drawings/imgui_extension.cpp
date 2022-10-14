#include <realtime/window.h>

#include "imgui_extension.h"

bool ImGui::InputVec3f(const char* label, vec3f& v, const char* format[3], ImGuiInputTextFlags flags) {
  if (format == NULL) {
    const char* format_default[] = {"%.3f", "%.3f", "%.3f"};
    format = format_default;
  }

  return InputScalarN(label, ImGuiDataType_Float, &v, 3, NULL, NULL, format, flags);
}

bool ImGui::InputScalarN(const char* label, ImGuiDataType data_type, void* v, int components, const void* step, const void* step_fast, const char* format[3], ImGuiInputTextFlags flags)
{
    ImGuiWindow* window = GetCurrentWindow();
    if (window->SkipItems)
        return false;

    ImGuiContext& g = *GImGui;
    bool value_changed = false;
    BeginGroup();
    PushID(label);
    PushMultiItemsWidths(components);
    size_t type_size = GDataTypeInfo[data_type].Size;
    for (int i = 0; i < components; i++)
    {
        PushID(i);
        value_changed |= InputScalar("", data_type, v, step, step_fast, format[i], flags);
        SameLine(0, g.Style.ItemInnerSpacing.x);
        PopID();
        PopItemWidth();
        v = (void*)((char*)v + type_size);
    }
    PopID();

    TextEx(label, FindRenderedTextEnd(label));
    EndGroup();
    return value_changed;
}