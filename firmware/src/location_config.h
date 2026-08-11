#pragma once

// 観測地の設定。
//
// 磁気偏角は必須。センサが返すのは磁方位、天体計算が返すのは真方位なので、
// これを入れないと日本では 7-9 度ずれる。自分の土地の値は NOAA の計算機
// (https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml) で得られる。
//
// 緯度経度は天体の位置計算に使う。GeoIP有効時は取得値で上書きされ、失敗時は
// NVSキャッシュ、さらにこの固定値へフォールバックする。GeoIPは概算なので、
// 正確な設置地点が分かる場合はwifi_config.hでGEOIP_ENABLEDをfalseにする。

// 千葉市付近
inline constexpr double kSiteLatitudeDegrees = 35.607;
inline constexpr double kSiteLongitudeEastDegrees = 140.106;

// 東経を正とする。日本は西偏なので負の値になる。
// 2026 年の千葉はおよそ西偏 7.9 度。
inline constexpr float kSiteDeclinationEast = -7.9F;
