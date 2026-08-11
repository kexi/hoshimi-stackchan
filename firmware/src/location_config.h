#pragma once

// 観測地の設定。
//
// 磁気偏角は必須。センサが返すのは磁方位、天体計算が返すのは真方位なので、
// これを入れないと日本では 7-9 度ずれる。自分の土地の値は NOAA の計算機
// (https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml) で得られる。
//
// 緯度経度は天体の位置計算に使う。数 km ずれても方位への影響は無視できるが、
// 100km 単位でずれると効いてくる。

// 東京駅付近
inline constexpr double kSiteLatitudeDegrees = 35.681236;
inline constexpr double kSiteLongitudeEastDegrees = 139.767125;

// 東経を正とする。日本は西偏なので負の値になる。
// 2026 年の東京はおよそ西偏 7.9 度。
inline constexpr float kSiteDeclinationEast = -7.9F;
