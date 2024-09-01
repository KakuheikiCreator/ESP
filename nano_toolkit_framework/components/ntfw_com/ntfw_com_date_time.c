/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :common date time library source file
 *
 * CREATED:2019/10/14 18:22:00
 * AUTHOR :Kakuheiki.Nakanohito
 *
 * DESCRIPTION:日付・時刻に関する共通ライブラリ
 *
 * CHANGE HISTORY:
 *
 * LAST MODIFIED BY:
 *
 *******************************************************************************
 *
 * Copyright (c) 2024 Kakuheiki.Nakanohito
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 *
 ******************************************************************************/
/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include "ntfw_com_date_time.h"

#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Function Prototypes                                         ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Functions                                                ***/
/******************************************************************************/
//==============================================================================
// ウェイト関数
//==============================================================================
/*******************************************************************************
 *
 * NAME: i64_dtm_delay_msec
 *
 * DESCRIPTION:ディレイ（ミリ秒単位）
 *
 * PARAMETERS:      Name            RW  Usage
 *   int64_t        i64_msec        R   ウェイト期間（単位：ミリ秒）
 *
 * RETURNS:
 *   int64_t:ディレイ後の時刻（単位：マイクロ秒）
 *
 ******************************************************************************/
int64_t i64_dtm_delay_msec(int64_t i64_msec) {
    return i64_dtm_delay_usec((i64_msec * 1000) - 4);
}

/*******************************************************************************
 *
 * NAME: i64_dtm_delay_usec
 *
 * DESCRIPTION:ディレイ（マイクロ秒単位）
 *
 * PARAMETERS:      Name            RW  Usage
 *   int64_t        i64_usec        R   ウェイト期間（単位：マイクロ秒）
 *
 * RETURNS:
 *   int64_t:ディレイ後の時刻（単位：マイクロ秒）
 *
 ******************************************************************************/
int64_t i64_dtm_delay_usec(int64_t i64_usec) {
    int64_t i64_now = esp_timer_get_time();
    int64_t i64_remaining = i64_usec - 4;
    int64_t i64_end  = i64_remaining + i64_now;
    int64_t i64_msec = i64_remaining / 1000;
    if (i64_msec >= DT_UTIL_DELAY_THRESHOLD_MS) {
        vTaskDelay(i64_msec / portTICK_PERIOD_MS);
    }
    while ((i64_now = esp_timer_get_time()) < i64_end);
    return i64_now;
}

/*******************************************************************************
 *
 * NAME: i64_dtm_delay_until_msec
 *
 * DESCRIPTION:指定時刻までディレイ（ミリ秒単位）
 *
 * PARAMETERS:      Name            RW  Usage
 *   int64_t        i64_msec        R   待ち合わせ時刻（単位：ミリ秒）
 *
 * RETURNS:
 *   int64_t:ディレイ後の時刻（単位：マイクロ秒）
 *
 ******************************************************************************/
int64_t i64_dtm_delay_until_msec(int64_t i64_msec) {
    return i64_dtm_delay_until_usec(i64_msec * 1000);
}

/*******************************************************************************
 *
 * NAME: i64_dtm_delay_until_usec
 *
 * DESCRIPTION:指定時刻までディレイ（マイクロ秒単位）
 *
 * PARAMETERS:      Name            RW  Usage
 *   int64_t        i64_usec        R   待ち合わせ時刻（単位：マイクロ秒）
 *
 * RETURNS:
 *   int64_t:ディレイ後の時刻（単位：マイクロ秒）
 *
 ******************************************************************************/
int64_t i64_dtm_delay_until_usec(int64_t i64_usec) {
    int64_t i64_time = i64_usec - esp_timer_get_time();
    if (i64_time <= 0) {
        return esp_timer_get_time();
    }
    return i64_dtm_delay_usec(i64_time);
}

/*******************************************************************************
 *
 * NAME: i64_dtm_wait_msec
 *
 * DESCRIPTION:ビジーウェイト処理（ミリ秒単位）
 *
 * PARAMETERS:      Name            RW  Usage
 *   int64_t        i64_msec        R   ウェイト期間（単位：ミリ秒）
 *
 * RETURNS:
 *   int64_t:ウェイト後の時刻（単位：マイクロ秒）
 *
 ******************************************************************************/
int64_t i64_dtm_wait_msec(int64_t i64_msec) {
    return i64_dtm_wait_usec((i64_msec * 1000) + 4);
}

/*******************************************************************************
 *
 * NAME: i64_dtm_wait_usec
 *
 * DESCRIPTION:ビジーウェイト処理（マイクロ秒単位）
 *
 * PARAMETERS:      Name            RW  Usage
 *   int64_t        i64_usec        R   ウェイト期間（単位：マイクロ秒）
 *
 * RETURNS:
 *   int64_t:ウェイト後の時刻（単位：マイクロ秒）
 *
 ******************************************************************************/
int64_t i64_dtm_wait_usec(int64_t i64_usec) {
    int64_t i64_end = esp_timer_get_time() + i64_usec - 8;
    int64_t i64_now;
    while ((i64_now = esp_timer_get_time()) < i64_end);
    return i64_now;
}

//==============================================================================
// チェック関数
//==============================================================================

/*******************************************************************************
 *
 * NAME: b_dtm_valid_date
 *
 * DESCRIPTION:日付チェック
 *   紀元前１年を０としたグレゴリオ暦の日付妥当性をチェックする
 *   グレゴリオ暦を成立前にも適用してうるう年判定を行う。
 *
 * PARAMETERS:      Name            RW  Usage
 *   int            i_year          R   年
 *   int            i_month         R   月
 *   int            i_day           R   日
 *
 * RETURNS:
 *   bool TRUE:チェックOK
 *
 ******************************************************************************/
bool b_dtm_valid_date(int i_year, int i_month, int i_day) {
    // 月の妥当性チェック
    if (i_month < 1 || i_month > 12) {
        return false;
    }
    uint8_t u8_days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    uint8_t u8_last_day = u8_days[i_month - 1];
    // うるう年の２月判定
    if(b_dtm_is_leap_year(i_year) && i_month ==  2) {
        u8_last_day++;
    }
    // 日の妥当性チェック
    return (i_day > 0 && i_day <= u8_last_day);
}

/*******************************************************************************
 *
 * NAME: b_dtm_valid_time
 *
 * DESCRIPTION:時刻チェック
 *
 * PARAMETERS:      Name            RW  Usage
 *   int            i_hour          R   時
 *   int            i_min           R   分
 *   int            i_sec           R   秒
 *
 * RETURNS:
 *   bool TRUE:チェックOK
 *
 ******************************************************************************/
bool b_dtm_valid_time(int i_hour, int i_min, int i_sec) {
    if (!b_dtm_valid_hour(i_hour)) {
        return false;
    }
    if (!b_dtm_valid_min(i_min)) {
        return false;
    }
    return b_dtm_valid_sec(i_sec);
}

/*******************************************************************************
 *
 * NAME: b_dtm_valid_hour
 *
 * DESCRIPTION:時チェック
 *
 * PARAMETERS:      Name            RW  Usage
 *   int            i_hour          R   時
 *
 * RETURNS:
 *   bool TRUE:チェックOK
 *
 ******************************************************************************/
bool b_dtm_valid_hour(int i_hour) {
    return (i_hour >= 0 && i_hour < 24);
}

/*******************************************************************************
 *
 * NAME: b_dtm_valid_min
 *
 * DESCRIPTION:分チェック
 *
 * PARAMETERS:      Name            RW  Usage
 *   int            i_min           R   分
 *
 * RETURNS:
 *   bool TRUE:チェックOK
 *
 ******************************************************************************/
bool b_dtm_valid_min(int i_min) {
    return (i_min >= 0 && i_min < 60);
}

/*******************************************************************************
 *
 * NAME: b_dtm_valid_sec
 *
 * DESCRIPTION:秒チェック
 *
 * PARAMETERS:      Name            RW  Usage
 *   int            i_sec           R   秒
 *
 * RETURNS:
 *   bool TRUE:チェックOK
 *
 ******************************************************************************/
bool b_dtm_valid_sec(int i_sec) {
    return (i_sec >= 0 && i_sec < 60);
}

/*******************************************************************************
 *
 * NAME: b_dtm_is_leap_year
 *
 * DESCRIPTION:うるう年チェック
 *   (紀元前１年を起算年（０年）として、次の条件でうるう年判定する
 *   (1)4で割り切れない年は平年
 *   (2)400で割り切れる年はうるう年
 *   (3)100で割り切れる年は平年
 *
 * PARAMETERS:      Name            RW  Usage
 *   int            i_year          R   年
 *
 * RETURNS:
 *   bool TRUE:うるう年
 *
 ******************************************************************************/
bool b_dtm_is_leap_year(int i_year) {
    // うるう年判定
    return !((i_year % 400 != 0 && i_year % 100 == 0) || i_year % 4 != 0);
}

//==============================================================================
// 変換関数
//==============================================================================

/*******************************************************************************
 *
 * NAME: i_dtm_date_to_days
 *
 * DESCRIPTION:変換関数（紀元1月1日からの経過日数への変換）
 *
 * PARAMETERS:      Name            RW  Usage
 *   inyt           i_year          R   年(0年は紀元前１年)
 *   uint8_t        u8_month        R   月(1-12)
 *   uint8_t        u8_day          R   日(1-31)
 *
 * RETURNS:
 *   int 変換結果（1年1月1日からの経過日数）
 *
 * NOTES:
 * None.
 ******************************************************************************/
int i_dtm_date_to_days(int i_year, uint8_t u8_month, uint8_t u8_day) {
    // うるう年を考慮しないで当年1月1日までの日数を算出
    int i_wk_total_days = (uint32_t)(i_year - 1) * 365;
    // 当年末までのうるう日数を加算
    i_wk_total_days = i_wk_total_days + i_year / 4 - i_year / 100 + i_year / 400;
    // 各月（1-12）の月初日までの経過日数
    uint32_t u32_conv_days[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    // 当年のうるう日を調整
    if (b_dtm_is_leap_year(i_year)) {
        u32_conv_days[2] = 60;
    }
    // 月日を加算
    i_wk_total_days = i_wk_total_days + u32_conv_days[u8_month - 1];
    i_wk_total_days = i_wk_total_days + u8_day - 1;
    return i_wk_total_days;
}

/*******************************************************************************
 *
 * NAME: s_dtm_day_to_date
 *
 * DESCRIPTION:変換関数（紀元1月1日からの経過日数から日付への変換）
 *
 * PARAMETERS:      Name            RW  Usage
 *   int            i_days          R   紀元1月1日からの経過日数
 *
 * RETURNS:
 *   tsDate 変換結果（西暦の日付）
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_date_t s_dtm_day_to_date(int i_days) {
    //==========================================================================
    // 対象日と初回起算日からの残り日数を算出
    //==========================================================================
    // 紀元前1年1月1日(うるう年）を基準日として、経過日数分経過した日を算出
    int i_target_day = i_days + 366;
    // 対象日までの残り日数
    int i_wk_days;
    // 起算年のうるう日数（かならず初回起算年はうるう年）
    int i_leap_days = 1;
    // 起算日の探索
    ts_date_t s_date;
    if (i_target_day >= 0) {
        // 起算年を算出
        s_date.i_year = (i_target_day / DT_UTIL_DAYS_400YEARS) * 400;
        // 残り日数を算出
        i_wk_days = i_target_day % DT_UTIL_DAYS_400YEARS;
    } else {
        // 起算年を算出
        int i_cnt = (i_target_day - DT_UTIL_DAYS_400YEARS + 1) / DT_UTIL_DAYS_400YEARS;
        s_date.i_year = i_cnt * 400;
        // 残り日数を算出
        i_wk_days = i_target_day - (i_cnt * DT_UTIL_DAYS_400YEARS);
        // ４００年単位で割り切れたのか判定
    }
    //==========================================================================
    // １００年単位の次の起算日を算出（対象日まで４００年未満）
    //==========================================================================
    // １００年単位の年数を算出
    while (i_wk_days >= (DT_UTIL_DAYS_100YEARS + i_leap_days)) {
        s_date.i_year += 100;
        i_wk_days -= (DT_UTIL_DAYS_100YEARS + i_leap_days);
        i_leap_days = 0;
    }
    //==========================================================================
    // ４年単位の次の起算日を算出（対象日まで１００年未満）
    //==========================================================================
    // １００で割れて４００で割り切れない年（４年間うるう年が無い）の対応
    if (i_leap_days == 0 && i_wk_days >= (DT_UTIL_DAYS_4YEARS - 1)) {
        s_date.i_year += 4;
        i_wk_days -= (DT_UTIL_DAYS_4YEARS - 1);
    }
    // ４年単位の年数を算出
    s_date.i_year += (i_wk_days / DT_UTIL_DAYS_4YEARS) * 4;
    i_wk_days %= DT_UTIL_DAYS_4YEARS;

    //==========================================================================
    // 対象年を算出（対象日まで４年未満）
    //==========================================================================
    // うるう年を考慮した事前処理
    int i_conv_days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    int i_days_per_year = 365;  // 年間日数
    // 起算年（必ずうるう年候補の年）のうるう年判定
    if (b_dtm_is_leap_year(s_date.i_year)) {
        i_conv_days[1]  = 29;
        i_days_per_year = 366;
    }
    // 対象年の算出
    while (i_wk_days >= i_days_per_year) {
        s_date.i_year++;
        i_wk_days -= i_days_per_year;
        i_days_per_year = 365;
        i_conv_days[1]  = 28;
    }
    //==========================================================================
    // 対象月を計算
    //==========================================================================
    // 月数
    uint8_t u8_idx;
    for (u8_idx = 0; i_wk_days >= i_conv_days[u8_idx]; u8_idx++) {
        i_wk_days -= i_conv_days[u8_idx];
    }
    s_date.i_month = u8_idx + 1;
    //==========================================================================
    // 対象日を計算
    //==========================================================================
    // 日数
    s_date.i_day = i_wk_days + 1;
    // 算出した日付を返却
    return s_date;
}

/******************************************************************************/
/***      Local Functions                                                   ***/
/******************************************************************************/

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
