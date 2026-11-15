// Замеры для отчёта: туннелирование, время шага, устойчивость, детерминизм
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include "physics.hpp"

using Clock = std::chrono::steady_clock;

namespace {
std::string g_out = "results";

struct Cfg {
  const char* name;
  Mode mode;
  double fastHz;
};
const Cfg kConfigs[] = {
    {"Discrete60", Mode::Discrete, 60}, {"CCD60", Mode::CCD, 60},
    {"MR_60", Mode::MultiRate, 60},     {"MR_120", Mode::MultiRate, 120},
    {"MR_240", Mode::MultiRate, 240},   {"MR_480", Mode::MultiRate, 480},
};

double pct(std::vector<double> v, double q) {
  if (v.empty()) return 0;
  std::sort(v.begin(), v.end());
  size_t i = static_cast<size_t>(q * (v.size() - 1));
  return v[i];
}

// Один «первый проход» снаряда через тонкую стенку сцены 1
// Возвращает: 1 — туннелирование, 0 — удар; x — где центр был в момент реакции;
// vout — скорость после
int firstPass(Mode mode, double hz, double speed, double x0, double e,
              double* xHit = nullptr, double* vOut = nullptr) {
  Params p;
  p.mode = mode;
  p.fastHz = hz;
  p.projSpeed = speed;
  p.scene1StartX = x0;
  p.restitution = e;
  p.trails = false;
  World w;
  w.build(1, p);
  for (int s = 0; s < 400; ++s) {
    w.step();
    const Body& b = w.dyn[0];
    if (w.tunnelCount > 0) return 1;
    if (b.v.x < 0) {
      if (xHit) *xHit = w.lastThinPen;
      if (vOut) *vOut = -b.v.x;
      return 0;
    }
    if (b.p.x > 700) return 1;
  }
  return 0;
}

// --------------------------------------------------------------------------
void expMinimal() {
  std::printf(
      "\n=== E1. Минимальный эксперимент (сцена 1, v=2000 px/c, стенка 10 px, "
      "r=5 px, x0=94.67) ===\n");
  std::printf("%-12s %-8s %-10s %-14s %-12s\n", "config", "fastHz", "result",
              "us/frame", "steps/frame");
  FILE* f = std::fopen((g_out + "/e1_minimal.csv").c_str(), "w");
  std::fprintf(f, "config,fastHz,tunnel,us_per_frame,fast_steps_per_frame\n");
  for (const auto& c : kConfigs) {
    int tun = firstPass(c.mode, c.fastHz, 2000, -1, 0.8);
    // Время: 200000 кадров (снаряд перезапускается каждую секунду)
    Params p;
    p.mode = c.mode;
    p.fastHz = c.fastHz;
    p.trails = false;
    World w;
    w.build(1, p);
    const int frames = 200000;
    auto t0 = Clock::now();
    for (int i = 0; i < frames; ++i) w.step();
    double us =
        std::chrono::duration<double, std::micro>(Clock::now() - t0).count() /
        frames;
    double spf = double(w.stats.fastSteps) / frames;
    std::printf("%-12s %-8.0f %-10s %-14.3f %-12.1f\n", c.name, c.fastHz,
                tun ? "TUNNEL" : "hit", us, spf);
    std::fprintf(f, "%s,%.0f,%d,%.4f,%.2f\n", c.name, c.fastHz, tun, us, spf);
  }
  std::fclose(f);
}

// --------------------------------------------------------------------------
void expPhaseSweep() {
  std::printf(
      "\n=== E2. Вероятность туннелирования по случайной фазе старта (5000 "
      "запусков) ===\n");
  const double v = 2000, w = 10, r = 5;
  std::printf("%-12s %-7s %-9s %-10s %-10s %-10s %-10s %-12s %-10s\n", "config",
              "Hz", "v*dt,px", "P_exp", "P_theory", "P_miss_th", "P_push_th",
              "depth_err,px", "v_out");
  FILE* f = std::fopen((g_out + "/e2_phase.csv").c_str(), "w");
  std::fprintf(f,
               "config,hz,step_px,p_exp,p_theory,p_miss_theory,p_push_theory,"
               "mean_depth_err,mean_vout\n");
  const int trials = 5000;
  for (const auto& c : kConfigs) {
    if (c.mode == Mode::Discrete) continue;  // Discrete60 == MR_60 в сцене 1
    double hz = c.mode == Mode::CCD ? 60 : c.fastHz;
    double step = v / hz;
    int tunnels = 0, hits = 0;
    double depthSum = 0, voutSum = 0;
    for (int i = 0; i < trials; ++i) {
      double x0 = 100.0 + step * (i + 0.5) / trials;  // равномерная сетка фаз
      double xh = 0, vo = 0;
      int t = firstPass(c.mode, c.fastHz, v, x0, 0.8, &xh, &vo);
      if (t)
        ++tunnels;
      else {
        ++hits;
        depthSum += xh;
        voutSum += vo;
      }
    }
    double pExp = double(tunnels) / trials;
    double pMiss =
        c.mode == Mode::CCD ? 0 : std::max(0.0, 1.0 - (w + 2 * r) / step);
    double pTot =
        c.mode == Mode::CCD ? 0 : std::max(0.0, 1.0 - (w / 2 + r) / step);
    double depth = hits ? depthSum / hits : 0, vout = hits ? voutSum / hits : 0;
    std::printf(
        "%-12s %-7.0f %-9.2f %-10.4f %-10.4f %-10.4f %-10.4f %-12.3f %-10.1f\n",
        c.name, hz, step, pExp, pTot, pMiss, pTot - pMiss, depth, vout);
    std::fprintf(f, "%s,%.0f,%.3f,%.5f,%.5f,%.5f,%.5f,%.4f,%.2f\n", c.name, hz,
                 step, pExp, pTot, pMiss, pTot - pMiss, depth, vout);
  }
  std::fclose(f);

  // Зависимость от скорости на 60 и 240 Гц (дискретно) и CCD
  FILE* g = std::fopen((g_out + "/e2_speed.csv").c_str(), "w");
  std::fprintf(g, "speed,p60,p60_theory,p240,p240_theory,pccd\n");
  for (double sp = 250; sp <= 8000; sp += 250) {
    int t60 = 0, t240 = 0, tc = 0;
    const int n = 1000;
    for (int i = 0; i < n; ++i) {
      double ph = (i + 0.5) / n;
      t60 += firstPass(Mode::MultiRate, 60, sp, 100 + ph * sp / 60, 0.8);
      t240 += firstPass(Mode::MultiRate, 240, sp, 100 + ph * sp / 240, 0.8);
      tc += firstPass(Mode::CCD, 60, sp, 100 + ph * sp / 60, 0.8);
    }
    auto th = [&](double hz) {
      return std::max(0.0, 1.0 - (w / 2 + r) / (sp / hz));
    };
    std::fprintf(g, "%.0f,%.4f,%.4f,%.4f,%.4f,%.4f\n", sp, t60 / double(n),
                 th(60), t240 / double(n), th(240), tc / double(n));
  }
  std::fclose(g);
  std::printf("  (кривая по скоростям -> e2_speed.csv)\n");
}

// --------------------------------------------------------------------------
struct RunResult {
  double avgMs, p95Ms, maxMs, fastPerFrame, toiPerFrame, capPerFrame;
  long long tunnels, wallHits;
  double meanSlowSpeed, maxSlowSpeed, maxPen;
  unsigned long long hash;
};

RunResult runScene2(Mode mode, double fastHz, int slowCount, int fastCount,
                    double seconds, double measureFrom,
                    std::vector<double>* perFrame = nullptr) {
  Params p;
  p.mode = mode;
  p.fastHz = fastHz;
  p.slowCount = slowCount;
  p.fastCount = fastCount;
  p.trails = false;
  World w;
  w.build(2, p);
  const int frames = static_cast<int>(seconds * 60);
  std::vector<double> ms;
  ms.reserve(frames);
  double speedSum = 0, speedMax = 0, penMax = 0;
  long long speedN = 0;
  for (int i = 0; i < frames; ++i) {
    auto t0 = Clock::now();
    w.step();
    ms.push_back(
        std::chrono::duration<double, std::milli>(Clock::now() - t0).count());
    if (w.simTime >= measureFrom) {
      for (const auto& b : w.dyn) {
        if (b.fast || !b.active) continue;
        double s = len(b.v);
        speedSum += std::fabs(b.v.y);
        ++speedN;
        speedMax = std::max(speedMax, s);
      }
      if (i % 10 == 0) penMax = std::max(penMax, w.maxPenetration());
    }
  }
  RunResult r{};
  double sum = 0;
  for (double x : ms) sum += x;
  r.avgMs = sum / frames;
  r.p95Ms = pct(ms, 0.95);
  r.maxMs = pct(ms, 1.0);
  r.fastPerFrame = double(w.stats.fastSteps) / frames;
  r.toiPerFrame = double(w.stats.toiEvents) / frames;
  r.capPerFrame = double(w.stats.toiCapHits) / frames;
  r.tunnels = w.tunnelCount;
  r.wallHits = w.wallHits;
  r.meanSlowSpeed = speedN ? speedSum / speedN : 0;
  r.maxSlowSpeed = speedMax;
  r.maxPen = penMax;
  r.hash = w.stateHash();
  if (perFrame) *perFrame = ms;
  return r;
}

void expScene2() {
  std::printf(
      "\n=== E3. Сцена 2: 6 неподвижных, 50 обычных, 5 снарядов, 20 с "
      "симуляции ===\n");
  std::printf("%-11s %-7s %-7s %-7s %-9s %-8s %-7s %-6s %-6s %-9s %-9s\n",
              "config", "avgMs", "p95Ms", "maxMs", "fast/fr", "TOI/fr",
              "cap/fr", "tunn", "hits", "maxSlowV", "maxPen");
  FILE* f = std::fopen((g_out + "/e3_scene2.csv").c_str(), "w");
  std::fprintf(f,
               "config,avg_ms,p95_ms,max_ms,fast_steps_per_frame,toi_per_frame,"
               "cap_per_frame,tunnels,"
               "wall_hits,max_slow_speed,max_pen\n");
  for (const auto& c : kConfigs) {
    // 3 прогона, берём минимальное среднее (меньше шум от ОС)
    RunResult best{};
    best.avgMs = 1e9;
    for (int k = 0; k < 3; ++k) {
      RunResult r = runScene2(c.mode, c.fastHz, 50, 5, 20, 0);
      if (r.avgMs < best.avgMs) best = r;
    }
    std::printf(
        "%-11s %-7.3f %-7.3f %-7.3f %-9.1f %-8.1f %-7.2f %-6lld %-6lld %-9.0f "
        "%-9.2f\n",
        c.name, best.avgMs, best.p95Ms, best.maxMs, best.fastPerFrame,
        best.toiPerFrame, best.capPerFrame, best.tunnels, best.wallHits,
        best.maxSlowSpeed, best.maxPen);
    std::fprintf(f, "%s,%.4f,%.4f,%.4f,%.2f,%.2f,%.3f,%lld,%lld,%.1f,%.3f\n",
                 c.name, best.avgMs, best.p95Ms, best.maxMs, best.fastPerFrame,
                 best.toiPerFrame, best.capPerFrame, best.tunnels,
                 best.wallHits, best.maxSlowSpeed, best.maxPen);
  }
  std::fclose(f);

  // Профиль времени по кадрам для графика
  FILE* g = std::fopen((g_out + "/e3_frames.csv").c_str(), "w");
  std::fprintf(g, "frame,Discrete60,CCD60,MR_240\n");
  std::vector<double> a, b, c;
  runScene2(Mode::Discrete, 60, 50, 5, 20, 0, &a);
  runScene2(Mode::CCD, 60, 50, 5, 20, 0, &b);
  runScene2(Mode::MultiRate, 240, 50, 5, 20, 0, &c);
  for (size_t i = 0; i < a.size(); ++i)
    std::fprintf(g, "%zu,%.4f,%.4f,%.4f\n", i, a[i], b[i], c[i]);
  std::fclose(g);
}

void expStability() {
  std::printf(
      "\n=== E4. Устойчивость кучи (без снарядов, 50 шаров, замер с 10 по 20 "
      "с) ===\n");
  std::printf("%-11s %-12s %-12s %-10s %-9s\n", "config", "mean|v_y|", "max|v|",
              "maxPen", "TOI/fr");
  FILE* f = std::fopen((g_out + "/e4_stability.csv").c_str(), "w");
  std::fprintf(
      f, "config,mean_abs_vy,max_speed,max_pen,toi_per_frame,cap_per_frame\n");
  for (const auto& c : kConfigs) {
    if (c.mode == Mode::MultiRate && c.fastHz != 240)
      continue;  // без снарядов MR == Discrete для кучи
    RunResult r = runScene2(c.mode, c.fastHz, 50, 0, 20, 10);
    std::printf("%-11s %-12.3f %-12.2f %-10.3f %-9.1f\n", c.name,
                r.meanSlowSpeed, r.maxSlowSpeed, r.maxPen, r.toiPerFrame);
    std::fprintf(f, "%s,%.4f,%.3f,%.4f,%.2f,%.3f\n", c.name, r.meanSlowSpeed,
                 r.maxSlowSpeed, r.maxPen, r.toiPerFrame, r.capPerFrame);
  }
  std::fclose(f);
}

void expDeterminism() {
  std::printf(
      "\n=== E5. Детерминизм: хэш состояния после 1200 шагов, два прогона + "
      "прогон с «рваным» кадром ===\n");
  FILE* f = std::fopen((g_out + "/e5_determinism.csv").c_str(), "w");
  std::fprintf(f, "config,hash_run1,hash_run2,hash_jitter,equal\n");
  for (const auto& c : kConfigs) {
    auto run = [&](bool jitter) {
      Params p;
      p.mode = c.mode;
      p.fastHz = c.fastHz;
      p.trails = false;
      World w;
      w.build(2, p);
      if (!jitter) {
        for (int i = 0; i < 1200; ++i) w.step();
        return w.stateHash();
      }
      // Аккумулятор с «рваными» кадрами 5..40 мс: шагов на кадр разное число,
      // но шаг физики фиксирован
      double acc = 0;
      uint32_t s = 777;
      while (w.stepIndex < 1200) {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        acc += 0.005 + 0.035 * ((s >> 8) / 16777216.0);
        while (acc >= w.dtSlow() && w.stepIndex < 1200) {
          w.step();
          acc -= w.dtSlow();
        }
      }
      return w.stateHash();
    };
    auto h1 = run(false), h2 = run(false), h3 = run(true);
    bool eq = h1 == h2 && h2 == h3;
    std::printf("%-11s %016llx %016llx %016llx %s\n", c.name,
                (unsigned long long)h1, (unsigned long long)h2,
                (unsigned long long)h3, eq ? "OK" : "DIFF");
    std::fprintf(f, "%s,%016llx,%016llx,%016llx,%d\n", c.name,
                 (unsigned long long)h1, (unsigned long long)h2,
                 (unsigned long long)h3, eq);
  }
  // Чувствительность: возмущение одной координаты на 1e-9 px
  {
    Params p;
    p.mode = Mode::Discrete;
    p.trails = false;
    World a;
    a.build(2, p);
    World b;
    b.build(2, p);
    b.dyn[0].p.x += 1e-9;
    std::fprintf(f, "\nstep,divergence_px\n");
    std::printf("  Чувствительность к возмущению 1e-9 px (Discrete60):");
    for (int i = 1; i <= 1200; ++i) {
      a.step();
      b.step();
      if (i % 120 == 0) {
        double m = 0;
        for (size_t k = 0; k < a.dyn.size(); ++k)
          m = std::max(m, len(a.dyn[k].p - b.dyn[k].p));
        std::fprintf(f, "%d,%.3e\n", i, m);
        std::printf(" t=%ds:%.1e", i / 60, m);
      }
    }
    std::printf("\n");
  }
  std::fclose(f);
}

void expScaling() {
  std::printf(
      "\n=== E6. Масштабирование: ms/frame от числа обычных объектов (5 "
      "снарядов, 10 с) ===\n");
  FILE* f = std::fopen((g_out + "/e6_scaling.csv").c_str(), "w");
  std::fprintf(f, "slow_count,Discrete60,CCD60,MR_240\n");
  std::printf("%-6s %-12s %-12s %-12s\n", "N", "Discrete60", "CCD60", "MR_240");
  for (int n : {10, 25, 50, 75, 100}) {
    double a = 1e9, b = 1e9, c = 1e9;
    for (int k = 0; k < 2; ++k) {
      a = std::min(a, runScene2(Mode::Discrete, 60, n, 5, 10, 0).avgMs);
      b = std::min(b, runScene2(Mode::CCD, 60, n, 5, 10, 0).avgMs);
      c = std::min(c, runScene2(Mode::MultiRate, 240, n, 5, 10, 0).avgMs);
    }
    std::printf("%-6d %-12.3f %-12.3f %-12.3f\n", n, a, b, c);
    std::fprintf(f, "%d,%.4f,%.4f,%.4f\n", n, a, b, c);
  }
  std::fclose(f);
}
}  // namespace

int runBench(int argc, char** argv) {
  if (argc > 1) g_out = argv[1];
  std::filesystem::create_directories(g_out);  // папка для CSV
  expMinimal();
  expPhaseSweep();
  expScene2();
  expStability();
  expDeterminism();
  expScaling();
  return 0;
}
