#pragma once

#include <raylib.h>
#include <string>
#include <vector>

// Renderer fuer kamerafeste 3D-Textlabels mit optionaler Kontur.
//
// Die Labels werden im 3D-Pass (innerhalb BeginMode3D/EndMode3D) gezeichnet:
// Jede Glyphe wird als kamerafestes, texturiertes Quad (raylib DrawBillboardPro)
// in der Bildebene aufgebaut, sodass der Z-Buffer die Texte wie jede andere
// Geometrie verdeckt. Fuer die Kontur wird dieselbe Glyphe mehrfach in einer
// Konturfarbe in 8 Richtungen versetzt gezeichnet (wie das ImGui-Overlay).
//
// Die Glyphen stammen aus dem raylib-Default-Font-Atlas. Da dessen
// GlyphInfo.advanceX == 0 ist, wird als Fortschritt die Glyphenbreite verwendet
// (so wie raylibs DrawTextEx).
class BillboardTextRenderer {
public:
    // Ein Textlabel an einem Weltanker (Text wird um den Anker zentriert).
    struct Item {
        Vector3 position;   // Ankerpunkt in Weltunits
        std::string text;
    };

    // Zeichnet alle Items. Erwartet einen offenen 3D-Kontext (BeginMode3D).
    //   worldHeight  : Zeilenhoehe in Weltunits
    //   color        : Textfarbe
    //   outlineColor : Konturfarbe
    //   outlinePx    : Konturstaerke in Font-Pixeln (0 = keine Kontur)
    void draw(const Camera& camera, const std::vector<Item>& items,
              float worldHeight, const Color& color, const Color& outlineColor,
              float outlinePx = 1.0f);
};
