#ifndef LV_CONF_H
#define LV_CONF_H

/* 屏幕分辨率 */
#define LV_HOR_RES_MAX          240
#define LV_VER_RES_MAX          320

/* 颜色深度（16位RGB565） */
#define LV_COLOR_DEPTH          16

/* 启用日志（调试用） */
#define LV_USE_LOG              1

#define LV_FONT_SIMSUN_16_CJK   1
#define LV_FONT_MONTSERRAT_16   1

/*Always set a default font*/
#define LV_FONT_DEFAULT &lv_font_simsun_16_cjk

/*Enable handling large font and/or fonts with a lot of characters.
 *The limit depends on the font size, font face and bpp.
 *Compiler error will be triggered if a font needs it.*/
#define LV_FONT_FMT_TXT_LARGE 1

/*Enables/disables support for compressed fonts.*/
#define LV_USE_FONT_COMPRESSED 1

#define LV_USE_INDEV 1

#endif // LV_CONF_H
