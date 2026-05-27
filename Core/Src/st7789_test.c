#include "st7789.h"
#include "main.h"

// ════════════════════════════════════════════════
//  TEST 1 — Backlight seul
//  Résultat attendu : écran blanc uniforme
// ════════════════════════════════════════════════
void Test1_Backlight(void)
{
    LCD_BLK_LOW();
    HAL_Delay(500);
    LCD_BLK_HIGH();   // → doit s'allumer en blanc
    HAL_Delay(1000);
}

// ════════════════════════════════════════════════
//  TEST 2 — Remplissage couleur unique
//  Résultat attendu : écran rouge plein
// ════════════════════════════════════════════════
void Test2_FillRed(void)
{
    ST7789_FillScreen(0xF800);   // Rouge RGB565
    HAL_Delay(1500);
}

// ════════════════════════════════════════════════
//  TEST 3 — Cycle de couleurs primaires
//  Résultat attendu : rouge → vert → bleu → blanc → noir
// ════════════════════════════════════════════════
void Test3_ColorCycle(void)
{
    uint16_t colors[] = {
        0xF800,  // Rouge
        0x07E0,  // Vert
        0x001F,  // Bleu
        0xFFFF,  // Blanc
        0x0000,  // Noir
        0xFFE0,  // Jaune
        0x07FF,  // Cyan
        0xF81F   // Magenta
    };

    for (int i = 0; i < 8; i++)
    {
        ST7789_FillScreen(colors[i]);
        HAL_Delay(800);
    }
}

// ════════════════════════════════════════════════
//  TEST 4 — Damier (vérifie adressage X et Y)
//  Résultat attendu : grille de 8×10 cases
//                     alternance rouge/bleu
// ════════════════════════════════════════════════
void Test4_Checkerboard(void)
{
    ST7789_FillScreen(0x0000);

    uint16_t cell_w = 30;   // 240 / 8 = 30px
    uint16_t cell_h = 32;   // 320 / 10 = 32px

    for (uint16_t row = 0; row < 10; row++)
    {
        for (uint16_t col = 0; col < 8; col++)
        {
            uint16_t color = ((row + col) % 2 == 0) ? 0xF800 : 0x001F;
            ST7789_FillRect(col * cell_w, row * cell_h,
                            cell_w - 1, cell_h - 1, color);
        }
    }
    HAL_Delay(2000);
}

// ════════════════════════════════════════════════
//  TEST 5 — Dégradé vertical (vérifie RGB565)
//  Résultat attendu : bandes vertes du haut (sombre)
//                     au bas (clair)
// ════════════════════════════════════════════════
void Test5_Gradient(void)
{
    for (uint16_t y = 0; y < 320; y++)
    {
        // Composante verte (6 bits) de 0 à 63
        uint8_t g = (uint8_t)((y * 63) / 319);
        uint16_t color = (uint16_t)(g << 5);  // RGB565 vert uniquement
        ST7789_FillRect(0, y, 240, 1, color);
    }
    HAL_Delay(2000);
}

// ════════════════════════════════════════════════
//  TEST 6 — Croix centrée (vérifie les coins)
//  Résultat attendu : croix blanche sur fond noir
//                     touchant les 4 bords
// ════════════════════════════════════════════════
void Test6_Cross(void)
{
    ST7789_FillScreen(0x0000);

    // Barre horizontale
    ST7789_FillRect(0,   155, 240, 10, 0xFFFF);
    // Barre verticale
    ST7789_FillRect(115,   0,  10, 320, 0xFFFF);

    // Coins marqués (rouge) pour vérifier l'alignement
    ST7789_FillRect(0,   0,   20, 20, 0xF800);  // Haut-gauche
    ST7789_FillRect(220, 0,   20, 20, 0x07E0);  // Haut-droite
    ST7789_FillRect(0,   300, 20, 20, 0x001F);  // Bas-gauche
    ST7789_FillRect(220, 300, 20, 20, 0xFFE0);  // Bas-droite

    HAL_Delay(3000);
}

// ════════════════════════════════════════════════
//  SÉQUENCE COMPLÈTE
// ════════════════════════════════════════════════
void ST7789_RunAllTests(void)
{
    //ST7789_Init();

    Test1_Backlight();
    HAL_Delay(500);
    Test2_FillRed();
    HAL_Delay(500);
    Test3_ColorCycle();
    HAL_Delay(500);
    Test4_Checkerboard();
    HAL_Delay(500);
    Test5_Gradient();
    HAL_Delay(500);
    Test6_Cross();
    HAL_Delay(500);

    // Fin : écran vert = succès
    ST7789_FillScreen(0x07E0);
}