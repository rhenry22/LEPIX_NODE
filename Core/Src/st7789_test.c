#include "st7789_test.h"
#include "main.h"
#include <stdio.h>

void Test1_Backlight(void) {
    HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_RESET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);
}

void Test2_FillRed(void)      {
    ST7789_Fill_Color(RED);HAL_Delay(1500);
}

void Test3_ColorCycle(void)   {
    uint16_t c[] = {RED,GREEN,BLUE,WHITE,BLACK,YELLOW,CYAN,MAGENTA};
    for(int i=0;i<8;i++){ ST7789_Fill_Color(c[i]); HAL_Delay(800); }
}
void Test4_Checkerboard(void) {
    ST7789_Fill_Color(BLACK);
    for(uint16_t r=0;r<10;r++)
        for(uint16_t c=0;c<8;c++)
            ST7789_Fill(c*30,r*28,c*30+29,r*28+27,((r+c)%2)?BLUE:RED);
    HAL_Delay(2000);
}
void Test5_Gradient(void) {
    for(uint16_t y=0;y<ST7789_HEIGHT;y++){
        uint16_t color=(uint16_t)(((y*63)/(ST7789_HEIGHT-1))<<5);
        ST7789_Fill(0,y,ST7789_WIDTH-1,y,color);
    }
    HAL_Delay(2000);
}
void Test6_Cross(void) {
    ST7789_Fill_Color(BLACK);
    ST7789_Fill(0,ST7789_HEIGHT/2-5,ST7789_WIDTH-1,ST7789_HEIGHT/2+5,WHITE);
    ST7789_Fill(ST7789_WIDTH/2-5,0,ST7789_WIDTH/2+5,ST7789_HEIGHT-1,WHITE);
    ST7789_Fill(0,0,20,20,RED);
    ST7789_Fill(ST7789_WIDTH-20,0,ST7789_WIDTH-1,20,GREEN);
    ST7789_Fill(0,ST7789_HEIGHT-20,20,ST7789_HEIGHT-1,BLUE);
    ST7789_Fill(ST7789_WIDTH-20,ST7789_HEIGHT-20,ST7789_WIDTH-1,ST7789_HEIGHT-1,YELLOW);
    HAL_Delay(3000);
}
void Test7_Text(void) {
    ST7789_Fill_Color(BLACK);
    ST7789_WriteString(10,10,  "ST7789 OK",  Font_16x26, WHITE,  BLACK);
    ST7789_WriteString(10,50,  "SPI2 HAL",   Font_16x26, GREEN,  BLACK);
    ST7789_WriteString(10,90,  "240x220",    Font_16x26, YELLOW, BLACK);
    ST7789_WriteString(10,130, "STM32F407",  Font_11x18, CYAN,   BLACK);
    HAL_Delay(3000);
}

void ST7789_RunAllTests(void) {

    Test1_Backlight();
    printf("RunAllTests: Test1 done\r\n");

    Test2_FillRed();
    printf("RunAllTests: Test2 done\r\n");

    Test3_ColorCycle();
    printf("RunAllTests: Test3 done\r\n");

    Test4_Checkerboard();
    printf("RunAllTests: Test4 done\r\n");

    Test5_Gradient();
    printf("RunAllTests: Test5 done\r\n");

    Test6_Cross();
    printf("RunAllTests: Test6 done\r\n");

    Test7_Text();
    printf("RunAllTests: Test7 done\r\n");

    ST7789_Fill_Color(GREEN);
    printf("RunAllTests: ALL TESTS PASSED\r\n");
}