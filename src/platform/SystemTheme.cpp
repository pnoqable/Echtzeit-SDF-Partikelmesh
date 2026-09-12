#include "SystemTheme.hpp"

#if defined(__APPLE__)
    #include <CoreFoundation/CoreFoundation.h>
#elif defined(_WIN32)
    #define WIN32_LEAN_AND_MEAN
    #include <windows.h>
#endif

namespace SystemTheme {

Theme currentTheme() {
#if defined(__APPLE__)
    // AppleInterfaceStyle ist im User-Defaults gespeichert ("Dark" oder nil)
    CFStringRef style = (CFStringRef)CFPreferencesCopyAppValue(
        CFSTR("AppleInterfaceStyle"), kCFPreferencesAnyApplication);
    bool dark = false;
    if (style != nullptr) {
        dark = (CFStringCompare(style, CFSTR("Dark"), kCFCompareCaseInsensitive) == kCFCompareEqualTo);
        CFRelease(style);
    }
    return dark ? Theme::Dark : Theme::Light;
#elif defined(_WIN32)
    // Registry-Wert "AppsUseLightTheme" (0 = dunkel, 1 = hell)
    DWORD value = 0;
    DWORD size = sizeof(value);
    LONG result = RegGetValueW(HKEY_CURRENT_USER,
        L"Software\\Microsoft\\Windows\\CurrentVersion\\Themes\\Personalize",
        L"AppsUseLightTheme", RRF_RT_REG_DWORD, nullptr, &value, &size);
    if (result == ERROR_SUCCESS && value == 0) return Theme::Dark;
    return Theme::Light;
#else
    return Theme::Light;
#endif
}

}