#include "physics.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kTouch = 0.05;     // допуск «касания», px
constexpr double kApproach = 1e-9;  // порог «сближаются»
constexpr size_t kTrailLen = 140;
constexpr size_t kHitsLen = 24;

// Переносимый детерминированный ГСЧ (std::uniform_*_distribution зависит от
// реализации STL).
struct XorShift32 {
  uint32_t s;
  explicit XorShift32(uint32_t seed) : s(seed ? seed : 1u) {}
  uint32_t next() {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
  }
  double uni() { return (next() >> 8) * (1.0 / 16777216.0); }  // [0,1)
};

double massOf(double r) { return kPi * r * r; }  // плотность 1 => m ~ площадь
}  // namespace

const char* modeName(Mode m) {
  switch (m) {
    case Mode::Discrete:
      return "Discrete";
    case Mode::CCD:
      return "CCD";
    case Mode::MultiRate:
      return "Multi-rate";
  }
  return "?";
}

// ---------------------------------------------------------------------------
//  Геометрия: перекрытие (дискретная проверка)
// ---------------------------------------------------------------------------
bool overlapCircleCircle(const Vec2& pa, double ra, const Vec2& pb, double rb,
                         Vec2& n, double& pen) {
  Vec2 d = pa - pb;
  double R = ra + rb;
  double dd = dot(d, d);
  if (dd >= R * R) return false;
  double dist = std::sqrt(dd);
  n = dist > 1e-9 ? d / dist : Vec2(0.0, -1.0);
  pen = R - dist;
  return true;
}

bool overlapCircleBox(const Vec2& c, double r, const Box& b, Vec2& n,
                      double& pen) {
  Vec2 q(clampd(c.x, b.mn.x, b.mx.x), clampd(c.y, b.mn.y, b.mx.y));
  Vec2 d = c - q;
  double dd = dot(d, d);
  if (dd > 1e-18) {  // центр снаружи коробки
    if (dd >= r * r) return false;
    double dist = std::sqrt(dd);
    n = d / dist;
    pen = r - dist;
    return true;
  }
  // Центр внутри коробки: выталкиваем по оси минимального проникновения.
  // ВАЖНО: именно здесь возникает «продавливание» сквозь тонкую стенку,
  // если за шаг центр успел уйти за её середину.
  double dl = c.x - b.mn.x, dr = b.mx.x - c.x, dt = c.y - b.mn.y,
         db = b.mx.y - c.y;
  double m = std::min(std::min(dl, dr), std::min(dt, db));
  if (m == dl)
    n = Vec2(-1, 0);
  else if (m == dr)
    n = Vec2(1, 0);
  else if (m == dt)
    n = Vec2(0, -1);
  else
    n = Vec2(0, 1);
  pen = r + m;
  return true;
}

// ---------------------------------------------------------------------------
//  Геометрия: время столкновения (CCD)
//  |d + w t| = R  =>  (w.w) t^2 + 2 (d.w) t + (d.d - R^2) = 0
// ---------------------------------------------------------------------------
bool toiCircleCircle(const Vec2& d, const Vec2& w, double R, double tmax,
                     double& t) {
  double dd = dot(d, d);
  double b = dot(d, w);                     // половина линейного коэффициента
  if (dd <= (R + kTouch) * (R + kTouch)) {  // уже касаются / перекрываются
    if (b < -kApproach) {
      t = 0.0;
      return true;
    }
    return false;
  }
  if (b >= 0.0) return false;  // расходятся
  double a = dot(w, w);
  if (a < 1e-12) return false;
  double c = dd - R * R;
  double disc = b * b - a * c;
  if (disc < 0.0) return false;  // промах
  // меньший корень t1 = (-b - sqrt(disc))/a, записанный устойчиво: t1 = c / (-b
  // + sqrt(disc))
  double tt = c / (-b + std::sqrt(disc));
  if (tt < 0.0) tt = 0.0;
  if (tt > tmax) return false;
  t = tt;
  return true;
}

bool toiCircleBox(const Vec2& c, const Vec2& v, double r, const Box& b,
                  double tmax, double& t, Vec2& n) {
  // 1) Уже касаемся?
  {
    Vec2 nn;
    double pen;
    if (overlapCircleBox(c, r + kTouch, b, nn, pen)) {
      if (dot(v, nn) < -kApproach) {
        t = 0.0;
        n = nn;
        return true;
      }
      return false;
    }
  }
  // 2) Луч центра против AABB, расширенного на r (метод слэбов, покомпонентно)
  Vec2 emn = b.mn - Vec2(r, r), emx = b.mx + Vec2(r, r);
  double t0 = -std::numeric_limits<double>::infinity();
  double t1 = std::numeric_limits<double>::infinity();
  int axis = -1;
  double nsign = 0.0;
  const double pc[2] = {c.x, c.y}, vc[2] = {v.x, v.y};
  const double lo[2] = {emn.x, emn.y}, hi[2] = {emx.x, emx.y};
  for (int k = 0; k < 2; ++k) {
    if (std::fabs(vc[k]) < 1e-12) {
      if (pc[k] < lo[k] || pc[k] > hi[k]) return false;
      continue;
    }
    double ta = (lo[k] - pc[k]) / vc[k], tb = (hi[k] - pc[k]) / vc[k];
    double s = -1.0;  // входим через «min»-грань => нормаль смотрит в минус
    if (ta > tb) {
      std::swap(ta, tb);
      s = 1.0;
    }
    if (ta > t0) {
      t0 = ta;
      axis = k;
      nsign = s;
    }
    t1 = std::min(t1, tb);
  }
  if (t0 > t1 || t1 < 0.0 || t0 > tmax) return false;
  double tEnter = std::max(t0, 0.0);
  Vec2 h = c + v * tEnter;
  bool cx = h.x < b.mn.x || h.x > b.mx.x;
  bool cy = h.y < b.mn.y || h.y > b.mx.y;
  if (cx && cy) {
    // 3) Попали в угловую зону: точное время — касание с окружностью радиуса r
    // вокруг угла
    Vec2 k(h.x < b.mn.x ? b.mn.x : b.mx.x, h.y < b.mn.y ? b.mn.y : b.mx.y);
    double tc;
    if (!toiCircleCircle(c - k, v, r, tmax, tc)) return false;
    t = tc;
    n = normalize(c + v * tc - k);
    return true;
  }
  if (axis < 0) return false;
  t = tEnter;
  n = axis == 0 ? Vec2(nsign, 0.0) : Vec2(0.0, nsign);
  return true;
}

// ---------------------------------------------------------------------------
//  Построение сцен
// ---------------------------------------------------------------------------
void World::build(int sceneId, const Params& p) {
  prm = p;
  scene = sceneId;
  dyn.clear();
  pegs.clear();
  boxes.clear();
  simTime = 0.0;
  stepIndex = 0;
  tunnelCount = 0;
  wallHits = 0;
  stats = StepStats{};
  XorShift32 rng(prm.seed);

  // Стены арены (толстые, за пределами экрана)
  boxes.push_back({{-60, -60}, {20, kHeight + 60}});
  boxes.push_back({{kWidth - 20, -60}, {kWidth + 60, kHeight + 60}});
  boxes.push_back({{-60, -60}, {kWidth + 60, 20}});
  boxes.push_back({{-60, kHeight - 20}, {kWidth + 60, kHeight + 60}});

  auto makeProjectile = [&](Vec2 pos, double timer, double period) {
    Body b;
    b.r = prm.projRadius;
    b.invMass = 1.0 / massOf(b.r);
    b.fast = true;
    b.active = false;
    b.launchPos = pos;
    b.launchVel = Vec2(prm.projSpeed, 0.0);
    b.p = b.prevP = pos;
    b.timer = timer;
    b.period = period;
    dyn.push_back(b);
  };

  const double hw = prm.wallThickness * 0.5;
  if (scene == 1) {
    // Сцена 1: минимальный эксперимент «снаряд против тонкой стенки»
    Box wall{{640 - hw, 120}, {640 + hw, 600}, true};
    boxes.push_back(wall);
    // x0 подобран так, чтобы при 60 Гц центр оказался в 628 px (за 2 px до
    // контакта), а следующим шагом — в 661.3 px (уже за стенкой): классический
    // пролёт
    double x0 = prm.scene1StartX >= 0.0 ? prm.scene1StartX
                                        : 628.0 - 16.0 * (2000.0 / 60.0);
    // Период перезапуска: снаряд должен успеть долететь до стенки и отскочить
    // при любой скорости
    const double period = std::max(1.0, (640.0 - x0) / prm.projSpeed + 0.6);
    makeProjectile({x0, 360.0}, 0.0, period);
    return;
  }

  // Сцена 2: смешанная система объектов
  boxes.push_back({{1000, 80}, {1000 + prm.wallThickness, 560}, true});

  // «Корзина» для кучи шаров (проверка устойчивости многослойного стека)
  boxes.push_back({{280, 430}, {290, kHeight - 20}, false});
  boxes.push_back({{770, 430}, {780, kHeight - 20}, false});

  const double rSlowMin = 12.0, rSlowMax = 24.0;
  const double rPeg =
      (rSlowMin + rSlowMax);  // = 2 x средний радиус обычных (36 px)
  const Vec2 pegPos[10] = {{420, 370}, {640, 370}, {150, 300},  {880, 250},
                           {150, 560}, {890, 560}, {1150, 200}, {1150, 560},
                           {860, 420}, {170, 130}};
  for (int i = 0; i < std::clamp(prm.staticCount, 0, 10); ++i) {
    Body s;
    s.p = s.prevP = pegPos[i];
    s.r = rPeg;
    s.invMass = 0.0;
    pegs.push_back(s);
  }

  const bool compact = prm.slowCount <= 54;
  const int cols = compact ? 9 : 17;
  const double x0 = compact ? 330 : 70;
  for (int i = 0; i < prm.slowCount; ++i) {
    Body b;
    b.r = rSlowMin + (rSlowMax - rSlowMin) * rng.uni();
    b.invMass = 1.0 / massOf(b.r);
    int c = i % cols, row = i / cols;
    b.p = Vec2(x0 + c * 50 + (rng.uni() - 0.5) * 6,
               50 + row * 50 + (rng.uni() - 0.5) * 6);
    b.v = Vec2((rng.uni() - 0.5) * 60, 0);
    b.prevP = b.p;
    dyn.push_back(b);
  }

  for (int i = 0; i < prm.fastCount; ++i) {
    double y =
        prm.fastCount > 1 ? 120 + i * (360.0 / (prm.fastCount - 1)) : 300.0;
    makeProjectile({45, y}, 0.5 + 0.37 * i, 2.0);
  }
}

int World::fastSubsteps() const {
  return std::max(1, static_cast<int>(std::lround(prm.fastHz / prm.slowHz)));
}

// ---------------------------------------------------------------------------
//  Шаг симуляции
// ---------------------------------------------------------------------------
void World::updateLaunchers(double dt) {
  for (auto& b : dyn) {
    if (!b.fast) continue;
    b.timer -= dt;
    if (b.timer <= 1e-12) {
      b.p = b.prevP = b.launchPos;
      b.v = b.launchVel;
      b.active = true;
      b.trail.clear();
      b.hits.clear();
      b.trail.push_back(b.p);
      b.timer += b.period;
    }
  }
}

void World::integrate(const std::vector<int>& idx, double dt, bool posToo) {
  const double g = scene == 1 ? 0.0 : prm.gravity;
  for (int i : idx) {
    Body& b = dyn[i];
    b.v.y += g * dt;              // полунеявный Эйлер: сначала скорость,
    if (posToo) b.p += b.v * dt;  // потом положение новой скоростью
  }
}

void World::advance(const std::vector<int>& idx, double t) {
  for (int i : idx) dyn[i].p += dyn[i].v * t;
}

// Импульсная реакция + (опционально) коррекция перекрытия. n — от b к a
bool World::resolve(Body& a, Body* b, const Vec2& n, double pen,
                    bool positional) {
  double ib = b ? b->invMass : 0.0;
  double im = a.invMass + ib;
  if (im <= 0.0) return false;
  Vec2 vrel = a.v - (b ? b->v : Vec2());
  double vn = dot(vrel, n);
  bool hit = false;
  if (vn < 0.0) {
    double e = (-vn < prm.restThreshold) ? 0.0 : prm.restitution;
    double j = -(1.0 + e) * vn / im;  // величина импульса
    a.v += n * (j * a.invMass);
    if (b) b->v -= n * (j * ib);
    hit = true;
  }
  if (positional && pen > prm.slop) {
    double corr = (pen - prm.slop) * prm.percent / im;
    a.p += n * (corr * a.invMass);
    if (b) b->p -= n * (corr * ib);
  }
  return hit;
}

void World::solveDiscrete(const std::vector<int>& A, int iters) {
  static const std::vector<int> kEmpty;
  solveDiscrete2(A, kEmpty, iters);
}

void World::solveDiscrete2(const std::vector<int>& A, const std::vector<int>& B,
                           int iters) {
  Vec2 n;
  double pen;
  for (int it = 0; it < iters; ++it) {
    for (size_t ii = 0; ii < A.size(); ++ii) {
      Body& a = dyn[A[ii]];
      for (auto& s : pegs) {
        ++stats.pairTests;
        if (overlapCircleCircle(a.p, a.r, s.p, s.r, n, pen)) {
          bool hit = resolve(a, nullptr, n, pen, true);
          if (hit && a.fast && prm.trails) {
            a.hits.push_back(a.p - n * a.r);
            if (a.hits.size() > kHitsLen) a.hits.pop_front();
          }
        }
      }
      for (auto& bx : boxes) {
        ++stats.pairTests;
        if (overlapCircleBox(a.p, a.r, bx, n, pen)) {
          bool hit = resolve(a, nullptr, n, pen, true);
          if (hit && a.fast && bx.thin) {
            ++wallHits;
            lastThinPen = pen;
            if (prm.trails) {
              a.hits.push_back(a.p - n * a.r);
              if (a.hits.size() > kHitsLen) a.hits.pop_front();
            }
          }
        }
      }
      for (size_t jj = ii + 1; jj < A.size(); ++jj) {
        Body& b = dyn[A[jj]];
        ++stats.pairTests;
        if (overlapCircleCircle(a.p, a.r, b.p, b.r, n, pen))
          resolve(a, &b, n, pen, true);
      }
      for (int jb : B) {
        Body& b = dyn[jb];
        ++stats.pairTests;
        if (overlapCircleCircle(a.p, a.r, b.p, b.r, n, pen))
          resolve(a, &b, n, pen, true);
      }
    }
  }
}

void World::stepCCD(const std::vector<int>& A, double dt) {
  integrate(A, dt,
            false);  // скорости — на весь шаг, положения двигаем по событиям

  // 0) Предварительно гасим уже существующие (покоящиеся) контакты — только
  // скорости
  //    Без этого каждая пара в куче шаров даёт событие t_c = 0 и съедает лимит
  //    событий
  {
    Vec2 n;
    double pen;
    for (int it = 0; it < prm.solverIters; ++it) {
      for (size_t ii = 0; ii < A.size(); ++ii) {
        Body& a = dyn[A[ii]];
        for (auto& s : pegs)
          if (overlapCircleCircle(a.p, a.r + kTouch, s.p, s.r, n, pen))
            resolve(a, nullptr, n, 0, false);
        for (auto& bx : boxes)
          if (!bx.thin && overlapCircleBox(a.p, a.r + kTouch, bx, n, pen))
            resolve(a, nullptr, n, 0, false);
        for (size_t jj = ii + 1; jj < A.size(); ++jj) {
          Body& b = dyn[A[jj]];
          if (overlapCircleCircle(a.p, a.r + kTouch, b.p, b.r, n, pen))
            resolve(a, &b, n, 0, false);
        }
      }
    }
  }

  double t = 0.0;
  long long events = 0;
  bool fastOnly =
      false;  // после упора в лимит продолжаем CCD только для снарядов
  while (true) {
    double rem = dt - t;
    if (rem <= 1e-12) break;

    // 2) Ищем самое раннее событие на [t, t+rem]
    double best = rem;
    int kind = -1, ia = -1, ib = -1;
    Vec2 bestN;
    for (size_t ii = 0; ii < A.size(); ++ii) {
      const Body& a = dyn[A[ii]];
      double tc;
      Vec2 nn;
      const bool skipStatic = fastOnly && !a.fast;
      for (size_t k = 0; k < pegs.size() && !skipStatic; ++k) {
        ++stats.pairTests;
        if (toiCircleCircle(a.p - pegs[k].p, a.v, a.r + pegs[k].r, best, tc) &&
            (kind < 0 || tc < best)) {
          best = tc;
          kind = 0;
          ia = A[ii];
          ib = static_cast<int>(k);
        }
      }
      for (size_t k = 0; k < boxes.size() && !skipStatic; ++k) {
        ++stats.pairTests;
        if (toiCircleBox(a.p, a.v, a.r, boxes[k], best, tc, nn) &&
            (kind < 0 || tc < best)) {
          best = tc;
          kind = 1;
          ia = A[ii];
          ib = static_cast<int>(k);
          bestN = nn;
        }
      }
      for (size_t jj = ii + 1; jj < A.size(); ++jj) {
        const Body& b = dyn[A[jj]];
        if (fastOnly && !a.fast && !b.fast) continue;
        ++stats.pairTests;
        if (toiCircleCircle(a.p - b.p, a.v - b.v, a.r + b.r, best, tc) &&
            (kind < 0 || tc < best)) {
          best = tc;
          kind = 2;
          ia = A[ii];
          ib = A[jj];
        }
      }
    }

    if (kind < 0) {
      advance(A, rem);
      break;
    }  // 5) событий нет — до конца шага
    if (!fastOnly && events >= prm.maxToiEvents) {
      ++stats.toiCapHits;
      fastOnly =
          true;  // медленные дальше — дискретно, снаряды — по-прежнему CCD
      continue;
    }
    if (fastOnly && events >= prm.maxToiEvents + 32) {
      advance(A, rem);
      break;
    }

    // 3) Переносим все тела к моменту t_c
    advance(A, best);
    t += best;
    ++events;

    // 4) Реакция на столкновение
    Body& a = dyn[ia];
    if (kind == 0) {
      resolve(a, nullptr, normalize(a.p - pegs[ib].p), 0.0, false);
      if (a.fast && prm.trails) {
        a.hits.push_back(a.p);
        if (a.hits.size() > kHitsLen) a.hits.pop_front();
      }
    } else if (kind == 1) {
      bool hit = resolve(a, nullptr, bestN, 0.0, false);
      if (hit && a.fast && boxes[ib].thin) {
        ++wallHits;
        lastThinPen = 0.0;
        if (prm.trails) {
          a.hits.push_back(a.p);
          if (a.hits.size() > kHitsLen) a.hits.pop_front();
        }
      }
    } else {
      Body& b = dyn[ib];
      resolve(a, &b, normalize(a.p - b.p), 0.0, false);
    }
  }
  stats.toiEvents += events;
  // Страховочный дискретный проход: убирает остаточные перекрытия (покоящиеся
  // контакты, упор в лимит событий, погрешности округления)
  solveDiscrete(A, prm.solverIters);
}

void World::afterFastStep(const std::vector<int>& F,
                          const std::vector<Vec2>& before) {
  for (size_t k = 0; k < F.size(); ++k) {
    Body& b = dyn[F[k]];
    const Vec2& p0 = before[k];
    for (const auto& bx : boxes) {
      if (!bx.thin) continue;
      double cx = 0.5 * (bx.mn.x + bx.mx.x);
      bool inY = p0.y > bx.mn.y && p0.y < bx.mx.y && b.p.y > bx.mn.y &&
                 b.p.y < bx.mx.y;
      if (inY && ((p0.x < cx) != (b.p.x < cx))) ++tunnelCount;
    }
    if (prm.trails) {
      b.trail.push_back(b.p);
      if (b.trail.size() > kTrailLen) b.trail.pop_front();
    }
  }
}

void World::step() {
  const double dt = dtSlow();
  updateLaunchers(dt);
  for (auto& b : dyn) b.prevP = b.p;

  std::vector<int> all, slow, fast;
  for (int i = 0; i < static_cast<int>(dyn.size()); ++i) {
    if (!dyn[i].active) continue;
    all.push_back(i);
    (dyn[i].fast ? fast : slow).push_back(i);
  }
  std::vector<Vec2> before;
  auto snap = [&]() {
    before.clear();
    for (int i : fast) before.push_back(dyn[i].p);
  };

  switch (prm.mode) {
    case Mode::Discrete:
      snap();
      integrate(all, dt, true);
      solveDiscrete(all, prm.solverIters);
      afterFastStep(fast, before);
      ++stats.fastSteps;
      break;
    case Mode::CCD:
      snap();
      stepCCD(all, dt);
      afterFastStep(fast, before);
      ++stats.fastSteps;
      break;
    case Mode::MultiRate: {
      // Медленная группа: один шаг dt = 1/slowHz
      integrate(slow, dt, true);
      solveDiscrete(slow, prm.solverIters);
      // Быстрая группа: n под-шагов h = dt/n (sub-stepping); медленные в это
      // время «заморожены», но получают импульсы от снарядов
      const int n = fastSubsteps();
      const double h = dt / n;
      for (int s = 0; s < n; ++s) {
        snap();
        integrate(fast, h, true);
        solveDiscrete2(fast, slow, prm.solverIters);
        afterFastStep(fast, before);
        ++stats.fastSteps;
      }
      break;
    }
  }
  ++stats.slowSteps;
  simTime += dt;
  ++stepIndex;
}

// ---------------------------------------------------------------------------
//  Диагностика
// ---------------------------------------------------------------------------
uint64_t World::stateHash() const {
  uint64_t h = 1469598103934665603ull;  // FNV-1a
  auto mix = [&](double d) {
    uint64_t u;
    std::memcpy(&u, &d, sizeof u);
    for (int i = 0; i < 8; ++i) {
      h ^= (u >> (8 * i)) & 0xFF;
      h *= 1099511628211ull;
    }
  };
  for (const auto& b : dyn) {
    mix(b.p.x);
    mix(b.p.y);
    mix(b.v.x);
    mix(b.v.y);
  }
  return h;
}

double World::maxPenetration() const {
  double m = 0.0;
  Vec2 n;
  double pen;
  for (size_t i = 0; i < dyn.size(); ++i) {
    if (!dyn[i].active) continue;
    for (const auto& s : pegs)
      if (overlapCircleCircle(dyn[i].p, dyn[i].r, s.p, s.r, n, pen))
        m = std::max(m, pen);
    for (const auto& bx : boxes)
      if (!bx.thin && overlapCircleBox(dyn[i].p, dyn[i].r, bx, n, pen))
        m = std::max(m, pen);
    for (size_t j = i + 1; j < dyn.size(); ++j)
      if (dyn[j].active &&
          overlapCircleCircle(dyn[i].p, dyn[i].r, dyn[j].p, dyn[j].r, n, pen))
        m = std::max(m, pen);
  }
  return m;
}
