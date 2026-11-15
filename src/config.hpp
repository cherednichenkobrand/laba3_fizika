#pragma once

//  ПАРАМЕТРЫ ПО УМОЛЧАНИЮ

namespace cfg {

// Коэффициент восстановления e: 1 — абсолютно упругий удар, 0 - абсолютно
// неупругий
inline constexpr double kRestitution = 0.8;

// Скорость снаряда, px/с (по заданию 2000)
inline constexpr double kProjectileSpeed = 2000.0;

// Частота «медленных» объектов (и всей физики в режимах Discrete / CCD), Гц
inline constexpr double kSlowHz = 60.0;

// Частота «быстрых» объектов в режиме Multi-rate, Гц (по заданию 240)
inline constexpr double kFastHz = 240.0;

// Ускорение свободного падения в сцене 2, px/с^2 (в сцене 1 гравитации нет)
inline constexpr double kGravity = 600.0;

// Геометрия тонкой стенки и снаряда.
inline constexpr double kWallThickness = 10.0;    // px
inline constexpr double kProjectileRadius = 5.0;  // px

// Состав сцены 2.
inline constexpr int kStaticCount =
    6;  // неподвижные (3..10), радиус = 2 x средний радиус обычных
inline constexpr int kSlowCount = 50;  // обычные (10..100)
inline constexpr int kFastCount = 5;   // снаряды (1..10)

// Ниже этой нормальной скорости удар считается «касанием» и e обнуляется
// (гасит микроподскоки в куче шаров), px/с
inline constexpr double kRestThreshold = 40.0;

// Итерации последовательных импульсов в дискретном решателе
inline constexpr int kSolverIterations = 4;

// Максимум TOI-событий за один шаг CCD
inline constexpr int kMaxToiEvents = 64;

// Seed генератора — одинаковый seed => одинаковая сцена (детерминизм)
inline constexpr unsigned kSeed = 12345u;

}  // namespace cfg
