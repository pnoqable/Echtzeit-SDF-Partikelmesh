#include "BillboardTextRenderer.hpp"

#include <raymath.h>
#include <rlgl.h>

namespace {

// Ein aufgeloestes Glyph-Quad: Quellrechteck im Font-Atlas plus Zentrum (Welt)
// und Groesse in Weltunits.
struct GlyphQuad {
    Rectangle rec;
    Vector3   pos;
    Vector2   size;
};

// Horizontaler Fortschritt (in Font-Pixeln) eines Zeichens. Der raylib-Default-
// Font setzt advanceX == 0; dann ist die Glyphenbreite der Fortschritt (so wie
// es raylibs DrawTextEx/DrawTextPro handhaben).
float glyphAdvancePx(const Font& font, int codepoint) {
    const GlyphInfo gi = GetGlyphInfo(font, codepoint);
    return (gi.advanceX > 0) ? static_cast<float>(gi.advanceX)
                             : GetGlyphAtlasRec(font, codepoint).width;
}

} // namespace

void BillboardTextRenderer::draw(const Camera& camera, const std::vector<Item>& items,
                                 float worldHeight, const Color& color,
                                 const Color& outlineColor, float outlinePx) {
    if (items.empty() || worldHeight <= 0.0f) return;

    const Font font = GetFontDefault();
    if (font.texture.id == 0 || font.baseSize <= 0) return;

    const float s = worldHeight / static_cast<float>(font.baseSize);

    // Bildebenen-Basis aus der View-Matrix (dieselbe, die DrawBillboardPro nutzt).
    const Matrix camM = GetCameraMatrix(camera);
    const Vector3 camRight = Vector3Normalize(Vector3{ camM.m0, camM.m4, camM.m8 });
    const Vector3 camUp    = Vector3Normalize(Vector3{ camM.m1, camM.m5, camM.m9 });

    constexpr float spacingPx = 1.0f;   // Abstand zwischen Zeichen (Font-Pixel)

    // Glyphen-Quads aufsammeln (Layout je Label, horizontal zentriert).
    std::vector<GlyphQuad> quads;
    for (const Item& item : items) {
        float totalPx = 0.0f;
        for (char ch : item.text) totalPx += glyphAdvancePx(font, static_cast<int>(ch)) + spacingPx;
        float penX = -totalPx * s * 0.5f;
        const float lineCenter = font.baseSize * s * 0.5f;
        for (char ch : item.text) {
            const int cp = static_cast<int>(ch);
            const GlyphInfo gi = GetGlyphInfo(font, cp);
            const Rectangle rec = GetGlyphAtlasRec(font, cp);
            GlyphQuad q;
            q.rec = rec;
            q.size = { rec.width * s, rec.height * s };
            const float cX = penX + (gi.offsetX + rec.width * 0.5f) * s;
            const float cY = -((gi.offsetY + rec.height * 0.5f) * s - lineCenter);
            q.pos = Vector3Add(item.position,
                Vector3Add(Vector3Scale(camRight, cX), Vector3Scale(camUp, cY)));
            quads.push_back(q);
            penX += (glyphAdvancePx(font, cp) + spacingPx) * s;
        }
    }
    if (quads.empty()) return;

    // Die Custom-Renderer dieses Projekts binden eigene Shader; fuer die
    // Billboards den raylib-Default-Shader setzen (inkl. Uniform-Locations, die
    // der Batch fuer die MVP-Matrix braucht). Depth-Test bleibt an (Verdeckung),
    // aber Depth-Writes aus, damit die transparenten Quad-Raender nichts verdecken.
    rlSetShader(rlGetShaderIdDefault(), rlGetShaderLocsDefault());
    rlDisableDepthMask();

    // Kontur: dieselbe Glyphe in der Konturfarbe in 8 Richtungen versetzt. Die
    // Verschiebung liegt in der Bildebene (camRight/camUp), die Tiefe bleibt
    // damit gleich -> kein Z-Fighting.
    if (outlinePx > 0.0f) {
        const float off = outlinePx * s;
        static const float kDirs[8][2] = {
            { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 },
            { -1, -1 }, { 1, -1 }, { -1, 1 }, { 1, 1 },
        };
        for (const GlyphQuad& q : quads) {
            const Vector2 org = { q.size.x * 0.5f, q.size.y * 0.5f };
            for (const auto& d : kDirs) {
                const Vector3 pos = Vector3Add(q.pos, Vector3Add(
                    Vector3Scale(camRight, d[0] * off),
                    Vector3Scale(camUp, d[1] * off)));
                // origin in Weltunits (siehe DrawBillboardRec).
                DrawBillboardPro(camera, font.texture, q.rec, pos, camUp, q.size, org, 0.0f, outlineColor);
            }
        }
    }

    // Farbiger Text oben drauf.
    for (const GlyphQuad& q : quads) {
        DrawBillboardPro(camera, font.texture, q.rec, q.pos, camUp, q.size,
            Vector2{ q.size.x * 0.5f, q.size.y * 0.5f }, 0.0f, color);
    }

    // Batch mit deaktivierter Depth-Maske abfliessen lassen, sonst koennten die
    // noch gepufferten Quad-Vertices nach dem Wiederaktivieren Tiefe schreiben.
    rlDrawRenderBatchActive();
    rlEnableDepthMask();
}
