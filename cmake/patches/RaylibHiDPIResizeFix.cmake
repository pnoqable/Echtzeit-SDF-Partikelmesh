# CMake-Script (ausgefuehrt via PATCH_COMMAND in CMakeLists.txt)
# Backport des raylib-master-Fixes #4834/#1982 nach raylib 5.5:
#   Nach einem Fenster-Resize unter Windows/Linux mit HiDPI ("FLAG_WINDOW_HIGHDPI")
#   liefert GLFW im WindowSizeCallback die physikalische Pixelgroesse statt der
#   logischen. raylib 5.5 schrieb diese direkt nach CORE.Window.screen.width/Height,
#   wodurch GetScreenWidth()/GetScreenHeight() nach dem Resize den falschen Wert
#   zurueckgaben (fehlende Division durch GetWindowScaleDPI()).
#
#   raylib master loest das per separatem FramebufferSizeCallback(), der die
#   echte Framebuffer-Groesse (physisch) als Render-Groesse nimmt und daraus die
#   logische Screen-Groesse ableitet. Dieser Backport wird hier als reiner
#   String-Ersatz auf die gecachte 5.5-Quelle angewendet (plattformneutrale
#   Alternative zu "patch", da unter Windows kein patch-Binary vorausgesetzt wird).
#
#   WICHTIG - OS-unterschiedliches Verhalten:
#   * Windows/Linux: GLFW-Frame-Callback liefert physische Pixel -> Fix aktiv,
#     WindowSizeCallback wird zum No-op.
#   * macOS: raylib 5.5 skaliert den Retina-Framebuffer selbst in SetupViewport()
#     und GLFW liefert im WindowSizeCallback die logische (Punkt-)Groesse.
#     macOS behaelt daher das Original-5.5-Verhalten; der FramebufferSizeCallback
#     bleibt registriert, aber als No-op.

# Pfad zur raylib-Quelle wird per -DRAYLIB_SOURCE_DIR=<dir> uebergeben
# (FetchContent definiert raylib_SOURCE_DIR zu diesem Zeitpunkt noch nicht).

if(NOT DEFINED RAYLIB_SOURCE_DIR)
  message(FATAL_ERROR "RaylibHiDPIResizeFix: RAYLIB_SOURCE_DIR nicht gesetzt.")
endif()

set(PATCH_FILE "${RAYLIB_SOURCE_DIR}/src/platforms/rcore_desktop_glfw.c")

if(NOT EXISTS "${PATCH_FILE}")
  message(FATAL_ERROR "RaylibHiDPIResizeFix: Ziel nicht gefunden: ${PATCH_FILE}")
endif()

file(READ "${PATCH_FILE}" _content)

# Idempotenz: Plattform-geguardete Version (v2) bereits angewendet?
string(FIND "${_content}" "platform-guarded (v2)" _marker_v2)
if(NOT _marker_v2 EQUAL -1)
  message(STATUS "RaylibHiDPIResizeFix: Patch (v2) bereits angewendet, ueberspringe (${PATCH_FILE}).")
  return()
endif()

# Alten v1-Backport (ohne OS-Guard) erkannt -> Abbruch, sonst Ergebnis falsch.
string(FIND "${_content}" "static void FramebufferSizeCallback(GLFWwindow *window, int width, int height)" _marker_v1)
if(NOT _marker_v1 EQUAL -1)
  message(FATAL_ERROR "RaylibHiDPIResizeFix: Alte Patch-Version (v1) erkannt. Bitte build/debug/_deps/raylib-src und raylib-subbuild loeschen und neu konfigurieren.")
endif()

set(_old_callback "static void WindowSizeCallback(GLFWwindow *window, int width, int height)
{
    // Reset viewport and projection matrix for new size
    SetupViewport(width, height);

    CORE.Window.currentFbo.width = width;
    CORE.Window.currentFbo.height = height;
    CORE.Window.resizedLastFrame = true;

    if (IsWindowFullscreen()) return;

    // Set current screen size

    CORE.Window.screen.width = width;
    CORE.Window.screen.height = height;

    // NOTE: Postprocessing texture is not scaled to new size
}")

set(_new_callback "static void WindowSizeCallback(GLFWwindow *window, int width, int height)
{
#if defined(__APPLE__)
    // macOS: GLFW liefert hier die logische (Punkt-)Groesse; die Retina-
    // Framebuffer-Skalierung uebernimmt raylib 5.5 selbst in SetupViewport().
    // Reset viewport and projection matrix for new size
    SetupViewport(width, height);

    CORE.Window.currentFbo.width = width;
    CORE.Window.currentFbo.height = height;
    CORE.Window.resizedLastFrame = true;

    if (IsWindowFullscreen()) return;

    // Set current screen size

    CORE.Window.screen.width = width;
    CORE.Window.screen.height = height;

    // NOTE: Postprocessing texture is not scaled to new size
#else
    // WARNING: Windows/Linux liefern hier bei HiDPI die physikalische
    // Pixelgroesse statt der logischen. Das eigentliche Resize-Handling
    // uebernimmt der FramebufferSizeCallback() (siehe unten).
    (void)width;
    (void)height;
#endif
}

// GLFW3 FramebufferSize Callback, runs when framebuffer is resized
// WARNING: If FLAG_WINDOW_HIGHDPI is set, WindowContentScaleCallback() is called before this function
// NOTE: Backported from raylib master (issue #4834 / #1982), platform-guarded (v2)
static void FramebufferSizeCallback(GLFWwindow *window, int width, int height)
{
    (void)window;
#if !defined(__APPLE__)
    // WARNING: On window minimization, callback is called with 0 values,
    // but internal screen values should not be changed, it breaks things
    if ((width == 0) || (height == 0)) return;

    // Reset viewport and projection matrix for new size
    // NOTE: This stores the render size (physical pixels) with CORE.Window.render
    SetupViewport(width, height);

    // Set render size
    CORE.Window.render.width = width;
    CORE.Window.render.height = height;
    CORE.Window.currentFbo.width = width;
    CORE.Window.currentFbo.height = height;
    CORE.Window.resizedLastFrame = true;

    if ((CORE.Window.flags & FLAG_FULLSCREEN_MODE) > 0)
    {
        // On fullscreen mode, strategy is ignoring high-dpi and
        // use the all available display size
        CORE.Window.screen.width = width;
        CORE.Window.screen.height = height;
        CORE.Window.screenScale = MatrixScale(1.0f, 1.0f, 1.0f);
        SetMouseScale(1.0f, 1.0f);
    }
    else // Window mode (including borderless window)
    {
        if ((CORE.Window.flags & FLAG_WINDOW_HIGHDPI) > 0)
        {
            // Set screen size to logical pixel size, considering content scaling
            Vector2 scaleDpi = GetWindowScaleDPI();
            CORE.Window.screen.width = (int)((float)width/scaleDpi.x);
            CORE.Window.screen.height = (int)((float)height/scaleDpi.y);
            CORE.Window.screenScale = MatrixScale(scaleDpi.x, scaleDpi.y, 1.0f);
            // On Windows/Linux, mouse coords need to be scaled into logical space
            SetMouseScale(1.0f/scaleDpi.x, 1.0f/scaleDpi.y);
        }
        else
        {
            // Set screen size to render size (physical pixel size)
            CORE.Window.screen.width = width;
            CORE.Window.screen.height = height;
        }
    }
#else
    // macOS: Framebuffer ist Retina-skalier (physisch); screen/render-Handling
    // bleibt vollstaendig im WindowSizeCallback().
    // NOTE: Callback bleibt registriert (no-op) fuer GLFW-Kompatibilitaet.
    (void)width;
    (void)height;
#endif
}")

string(FIND "${_content}" "${_old_callback}" _pos)
if(_pos EQUAL -1)
  message(FATAL_ERROR "RaylibHiDPIResizeFix: WindowSizeCallback-Block nicht gefunden (Patch evtl. schon angewendet oder raylib-Quelle geaendert).")
endif()

string(REPLACE "${_old_callback}" "${_new_callback}" _content "${_content}")

set(_old_register "glfwSetWindowSizeCallback(platform.handle, WindowSizeCallback);      // NOTE: Resizing not allowed by default!")
set(_new_register "glfwSetWindowSizeCallback(platform.handle, WindowSizeCallback);      // NOTE: Resizing not allowed by default!
    glfwSetFramebufferSizeCallback(platform.handle, FramebufferSizeCallback);")

string(FIND "${_content}" "${_old_register}" _pos_reg)
if(_pos_reg EQUAL -1)
  message(FATAL_ERROR "RaylibHiDPIResizeFix: Callback-Registrierung nicht gefunden.")
endif()

string(REPLACE "${_old_register}" "${_new_register}" _content "${_content}")

set(_old_declare "static void WindowSizeCallback(GLFWwindow *window, int width, int height);                 // GLFW3 WindowSize Callback, runs when window is resized")
set(_new_declare "static void WindowSizeCallback(GLFWwindow *window, int width, int height);                 // GLFW3 WindowSize Callback, runs when window is resized
static void FramebufferSizeCallback(GLFWwindow *window, int width, int height);            // GLFW3 FramebufferSize Callback, runs when window framebuffer is resized")

string(FIND "${_content}" "${_old_declare}" _pos_decl)
if(_pos_decl EQUAL -1)
  message(FATAL_ERROR "RaylibHiDPIResizeFix: Forward-Declaration nicht gefunden.")
endif()

string(REPLACE "${_old_declare}" "${_new_declare}" _content "${_content}")

file(WRITE "${PATCH_FILE}" "${_content}")

message(STATUS "RaylibHiDPIResizeFix: FramebufferSizeCallback-Backport (v2, plattform-geguardet) auf raylib 5.5 angewendet (${PATCH_FILE}).")