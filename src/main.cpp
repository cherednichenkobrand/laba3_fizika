

#include <SFML/Graphics.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

#include "physics.hpp"

using Clock = std::chrono::steady_clock;

int runBench(int argc, char** argv);

#ifdef _WIN32
extern "C" __declspec(dllimport) int __stdcall SetConsoleOutputCP(unsigned int);
#endif

namespace {

sf::String U(const std::string& s) {
  return sf::String::fromUtf8(s.begin(), s.end());
}

std::string fmt(const char* f, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof buf, f, ap);
  va_end(ap);
  return buf;
}

const double kHzOptions[] = {60, 120, 240, 480};
const double kScaleOptions[] = {1.0, 0.25, 0.1, 0.02};

struct App {
  World world;
  Params prm;
  int scene = 1;
  double timeScale = 1.0;
  bool paused = false, showHud = true, showTrails = true;
  double acc = 0.0;
  std::deque<double> msHist;    // время физики за кадр
  std::deque<double> fastHist;  // шагов быстрой группы за кадр
  std::deque<double> slowHist;
  std::deque<double> toiHist;
  std::string banner;
  sf::Font font;

  void rebuild() {
    prm.trails = true;
    world.build(scene, prm);
    acc = 0.0;
    msHist.clear();
    fastHist.clear();
    slowHist.clear();
    toiHist.clear();
  }

  // Один кадр: аккумулятор фиксированного шага
  void advance(double frameDt) {
    world.stats = StepStats{};
    auto t0 = Clock::now();
    if (!paused) {
      acc += std::min(frameDt, 0.25) * timeScale;  // защита от «спирали смерти»
      const double dt = world.dtSlow();
      while (acc >= dt) {
        world.step();
        acc -= dt;
      }
    }
    double ms =
        std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    auto push = [](std::deque<double>& d, double v) {
      d.push_back(v);
      if (d.size() > 60) d.pop_front();
    };
    push(msHist, ms);
    push(fastHist, double(world.stats.fastSteps));
    push(slowHist, double(world.stats.slowSteps));
    push(toiHist, double(world.stats.toiEvents));
  }
  static double avg(const std::deque<double>& d) {
    if (d.empty()) return 0;
    double s = 0;
    for (double x : d) s += x;
    return s / d.size();
  }

  void draw(sf::RenderTarget& rt) {
    rt.clear(sf::Color(21, 24, 29));
    const double alpha =
        paused ? 1.0 : std::clamp(acc / world.dtSlow(), 0.0, 1.0);

    for (const auto& b : world.boxes) {
      sf::RectangleShape r(
          sf::Vector2f(float(b.mx.x - b.mn.x), float(b.mx.y - b.mn.y)));
      r.setPosition(float(b.mn.x), float(b.mn.y));
      r.setFillColor(b.thin ? sf::Color(232, 197, 71) : sf::Color(58, 63, 71));
      rt.draw(r);
    }
    for (const auto& s : world.pegs) {
      sf::CircleShape c(float(s.r), 48);
      c.setOrigin(float(s.r), float(s.r));
      c.setPosition(float(s.p.x), float(s.p.y));
      c.setFillColor(sf::Color(91, 101, 115));
      c.setOutlineColor(sf::Color(140, 150, 165));
      c.setOutlineThickness(2);
      rt.draw(c);
    }
    // Следы снарядов: точка = положение центра в конце шага физики
    if (showTrails) {
      for (const auto& b : world.dyn) {
        if (!b.fast || !b.active) continue;
        sf::VertexArray line(sf::LineStrip);
        size_t n = b.trail.size(), k = 0;
        for (const auto& p : b.trail) {
          sf::Uint8 a = sf::Uint8(110 + 145.0 * (k + 1) / n);
          line.append(sf::Vertex(sf::Vector2f(float(p.x), float(p.y)),
                                 sf::Color(255, 160, 110, a / 2)));
          sf::CircleShape d(3.5f, 12);
          d.setOrigin(3.5f, 3.5f);
          d.setPosition(float(p.x), float(p.y));
          d.setFillColor(sf::Color(255, 214, 160, a));
          rt.draw(d);
          ++k;
        }
        rt.draw(line);
        for (const auto& h : b.hits) {
          sf::CircleShape ring(7.f, 20);
          ring.setOrigin(7.f, 7.f);
          ring.setPosition(float(h.x), float(h.y));
          ring.setFillColor(sf::Color::Transparent);
          ring.setOutlineColor(sf::Color(80, 230, 120));
          ring.setOutlineThickness(2);
          rt.draw(ring);
        }
      }
    }
    for (const auto& b : world.dyn) {
      if (!b.active) continue;
      Vec2 p = b.prevP + (b.p - b.prevP) * alpha;  // интерполяция отрисовки
      if (b.fast) {
        sf::CircleShape glow(float(b.r + 5), 24);
        glow.setOrigin(float(b.r + 5), float(b.r + 5));
        glow.setPosition(float(p.x), float(p.y));
        glow.setFillColor(sf::Color(255, 90, 54, 70));
        rt.draw(glow);
      }
      sf::CircleShape c(float(b.r), b.fast ? 20 : 32);
      c.setOrigin(float(b.r), float(b.r));
      c.setPosition(float(p.x), float(p.y));
      if (b.fast)
        c.setFillColor(sf::Color(255, 90, 54));
      else {
        int t = int(255 * (b.r - 12) / 12.0);
        c.setFillColor(sf::Color(sf::Uint8(60 + t / 5), sf::Uint8(150 - t / 4),
                                 sf::Uint8(225)));
        c.setOutlineColor(sf::Color(20, 40, 70));
        c.setOutlineThickness(1);
      }
      rt.draw(c);
    }

    if (showHud) {
      std::string modeStr = modeName(prm.mode);
      if (prm.mode == Mode::MultiRate)
        modeStr +=
            fmt(" (обычные %.0f Гц / снаряды %.0f Гц)", prm.slowHz, prm.fastHz);
      else
        modeStr += fmt(" (все объекты %.0f Гц)", prm.slowHz);
      std::string hud = fmt("Сцена %d   Режим: %s\n", scene, modeStr.c_str());
      hud += fmt("e = %.2f   v снаряда = %.0f px/с   масштаб времени x%.2f%s\n",
                 prm.restitution, prm.projSpeed, timeScale,
                 paused ? "   [ПАУЗА]" : "");
      double steps = avg(slowHist);
      double msPerStep = steps > 0 ? avg(msHist) / steps : 0;
      hud +=
          fmt("физика: %.4f ms/кадр (%.4f ms на шаг 60 Гц)   шагов "
              "снарядов/кадр: %.2f",
              avg(msHist), msPerStep, avg(fastHist));
      if (prm.mode == Mode::CCD)
        hud += fmt("   TOI-событий/кадр: %.1f", avg(toiHist));
      hud += fmt(
          "\nТуннелирований: %lld   Ударов о жёлтую стенку: %lld   t = %.2f с",
          world.tunnelCount, world.wallHits, world.simTime);
      sf::Text t(U(hud), font, 16);
      t.setPosition(30, 30);
      t.setFillColor(sf::Color(230, 230, 230));
      sf::FloatRect bb = t.getGlobalBounds();
      sf::RectangleShape bg(sf::Vector2f(bb.width + 16, bb.height + 14));
      bg.setPosition(bb.left - 8, bb.top - 7);
      bg.setFillColor(sf::Color(0, 0, 0, 150));
      rt.draw(bg);
      rt.draw(t);

      sf::Text keys(U("1/2 сцена  M режим  F частота снарядов  [ ] e  Up/Down "
                      "скорость  T замедление  "
                      "Space пауза  N шаг  R рестарт  C следы  H HUD"),
                    font, 13);
      keys.setPosition(30, 2);
      keys.setFillColor(sf::Color(170, 170, 170));
      rt.draw(keys);
    }
    if (!banner.empty()) {
      sf::Text t(U(banner), font, 24);
      t.setStyle(sf::Text::Bold);
      sf::FloatRect bb = t.getLocalBounds();
      t.setPosition(float(World::kWidth - bb.width) / 2.f,
                    World::kHeight - 100.f);
      sf::RectangleShape bg(sf::Vector2f(bb.width + 30, bb.height + 22));
      bg.setPosition(t.getPosition().x - 15, t.getPosition().y - 6);
      bg.setFillColor(sf::Color(0, 0, 0, 190));
      bg.setOutlineColor(sf::Color(232, 197, 71));
      bg.setOutlineThickness(2);
      rt.draw(bg);
      t.setFillColor(sf::Color(255, 240, 200));
      rt.draw(t);
    }
  }
};

Mode parseMode(const std::string& s) {
  if (s == "ccd") return Mode::CCD;
  if (s == "multirate" || s == "mr") return Mode::MultiRate;
  return Mode::Discrete;
}

template <class T, size_t N>
T nextOption(const T (&arr)[N], T cur) {
  for (size_t i = 0; i < N; ++i) {
    if (std::fabs(arr[i] - cur) < 1e-9) return arr[(i + 1) % N];
  }
  return arr[0];
}

bool loadFont(sf::Font& f, const char* argv0) {
  std::string exe(argv0), dir = exe.substr(0, exe.find_last_of("/\\") + 1);
  for (std::string p :
       {dir + "assets/DejaVuSans.ttf", std::string("assets/DejaVuSans.ttf"),
        dir + "../assets/DejaVuSans.ttf"})
    if (f.loadFromFile(p)) return true;
  return false;
}

// Записанное демо: перед каждой сценой — титр на чёрном фоне (что показываем),
// потом сама симуляция. Кадры PNG -> ffmpeg собирает видео/GIF
struct Segment {
  int scene;
  Mode mode;
  double fastHz;
  double scale;    // замедление времени во время показа
  double seconds;  // длительность показа сцены, с
  double warmup;   // сколько секунд симуляции прокрутить до записи
  const char* title;
  const char* line1;
  const char* line2;
  const char* banner;  // короткая подпись во время сцены
};

// Титр: чёрный экран, номер пункта, заголовок и две строки пояснения.
void drawCard(sf::RenderTarget& rt, const sf::Font& font, int idx, int total,
              const Segment& s, float alpha) {
  rt.clear(sf::Color::Black);
  const sf::Uint8 a = sf::Uint8(255 * alpha);
  auto line = [&](const std::string& str, unsigned size, float y, sf::Color c,
                  bool bold) {
    sf::Text t(U(str), font, size);
    if (bold) t.setStyle(sf::Text::Bold);
    c.a = a;
    t.setFillColor(c);
    sf::FloatRect lb = t.getLocalBounds();
    t.setPosition(std::round((float(World::kWidth) - lb.width) / 2.f - lb.left),
                  y);
    rt.draw(t);
  };
  line(fmt("Пункт %d из %d", idx, total), 22, 230, sf::Color(232, 197, 71),
       false);
  line(s.title, 40, 280, sf::Color(240, 240, 240), true);
  line(s.line1, 24, 370, sf::Color(190, 190, 190), false);
  line(s.line2, 24, 408, sf::Color(190, 190, 190), false);
}

int runDemo(App& app, const std::string& dir) {
  const Segment segs[] = {
      {1, Mode::Discrete, 60, 0.1, 4.5, 0.0, "Без CCD: дискретная проверка, 60 Гц",
       "Снаряд 2000 px/с, стенка 10 px. За шаг он пролетает 33 px,",
       "в начале и в конце шага пересечения нет => туннелирование",
       "Discrete 60 Гц: пролёт сквозь стенку"},
      {1, Mode::CCD, 60, 0.1, 4.5, 0.0, "С CCD, те же 60 Гц",
       "Ищем момент удара t_c внутри шага (t_c = 0,001 с),",
       "переносим снаряд к стенке и считаем отскок: v' = e·v = 1600 px/с",
       "CCD 60 Гц: удар найден, отскок"},
      {1, Mode::MultiRate, 240, 0.1, 3.5, 0.0, "Multi-rate: снаряд на 240 Гц",
       "4 шага снаряда на один шаг 60 Гц, путь за шаг 8,3 px < 10 px,",
       "поэтому удар ловится даже обычной дискретной проверкой",
       "Multi-rate, снаряд 240 Гц: отскок"},
      {2, Mode::MultiRate, 60, 1.0, 5.0, 4.0, "Система объектов, снаряды 60 Гц",
       "6 неподвижных кругов, 50 шаров (60 Гц), 5 снарядов (60 Гц).",
       "Снаряды пролетают жёлтую стенку",
       "Снаряды 60 Гц: туннелирование"},
      {2, Mode::MultiRate, 240, 1.0, 5.0, 4.0, "Та же система, снаряды 240 Гц",
       "Шары по-прежнему 60 Гц, снаряды 240 Гц (4 под-шага на кадр).",
       "Снаряды отскакивают от стенки",
       "Снаряды 240 Гц: отскок"},
  };
  const int total = int(sizeof segs / sizeof segs[0]);
  sf::RenderTexture rt;
  if (!rt.create(unsigned(World::kWidth), unsigned(World::kHeight))) return 1;
  const double fps = 60.0, cardSec = 2.5, fadeSec = 0.4;
  int frame = 0;
  auto save = [&]() {
    rt.display();
    char name[64];
    std::snprintf(name, sizeof name, "/f%05d.png", frame++);
    rt.getTexture().copyToImage().saveToFile(dir + name);
  };
  for (int k = 0; k < total; ++k) {
    const Segment& s = segs[k];
    // 1) Титр с плавным появлением и исчезновением текста
    const int cardFrames = int(cardSec * fps);
    for (int i = 0; i < cardFrames; ++i) {
      double t = i / fps, alpha = 1.0;
      if (t < fadeSec) alpha = t / fadeSec;
      if (t > cardSec - fadeSec) alpha = (cardSec - t) / fadeSec;
      drawCard(rt, app.font, k + 1, total, s, float(std::clamp(alpha, 0.0, 1.0)));
      save();
    }
    // 2) Сцена
    app.scene = s.scene;
    app.prm.mode = s.mode;
    app.prm.fastHz = s.fastHz;
    app.rebuild();
    app.timeScale = 1.0;
    app.banner.clear();
    for (int i = 0; i < int(s.warmup * 60); ++i) app.advance(1.0 / 60);
    app.timeScale = s.scale;
    app.banner = s.banner;
    for (int i = 0; i < int(s.seconds * fps); ++i) {
      app.advance(1.0 / fps);
      app.draw(rt);
      save();
    }
  }
  app.banner.clear();
  std::printf("demo: %d frames\n", frame);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
#ifdef _WIN32
  SetConsoleOutputCP(65001);  // кириллица в консоли Windows
#endif
  // Режим замеров: physics2d --bench [папка]  -> таблицы в консоль и CSV в
  // results/
  if (argc > 1 && std::strcmp(argv[1], "--bench") == 0)
    return runBench(argc - 1, argv + 1);

  App app;
  app.prm = Params{};
  std::string demoDir, shotFile;
  double shotTime = 0.5, shotScale = 1.0;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&]() {
      return i + 1 < argc ? std::string(argv[++i]) : std::string();
    };
    if (a == "--scene")
      app.scene = std::stoi(next());
    else if (a == "--mode")
      app.prm.mode = parseMode(next());
    else if (a == "--fast-hz")
      app.prm.fastHz = std::stod(next());
    else if (a == "--e")
      app.prm.restitution = std::stod(next());
    else if (a == "--speed")
      app.prm.projSpeed = std::stod(next());
    else if (a == "--slow")
      app.prm.slowCount = std::stoi(next());
    else if (a == "--fast")
      app.prm.fastCount = std::stoi(next());
    else if (a == "--demo")
      demoDir = next();
    else if (a == "--shot")
      shotFile = next();
    else if (a == "--time")
      shotTime = std::stod(next());
    else if (a == "--banner")
      app.banner = next();
    else if (a == "--scale")
      shotScale = std::stod(next());
  }
  if (!loadFont(app.font, argv[0])) {
    std::fprintf(stderr, "Не найден assets/DejaVuSans.ttf\n");
    return 1;
  }
  app.rebuild();

  if (!demoDir.empty()) return runDemo(app, demoDir);
  if (!shotFile.empty()) {  // скриншот в момент времени симуляции --time
    sf::RenderTexture rt;
    if (!rt.create(unsigned(World::kWidth), unsigned(World::kHeight))) return 1;
    app.timeScale = shotScale;
    while (app.world.simTime + 1e-9 < shotTime)
      app.advance(1.0 / 60 / shotScale);
    app.acc = app.world.dtSlow();  // рисуем ровно состояние конца шага
    app.draw(rt);
    rt.display();
    rt.getTexture().copyToImage().saveToFile(shotFile);
    return 0;
  }

  sf::RenderWindow win(
      sf::VideoMode(unsigned(World::kWidth), unsigned(World::kHeight)),
      "2D rigid body: CCD vs Multi-rate",
      sf::Style::Titlebar | sf::Style::Close);
  win.setVerticalSyncEnabled(true);
  win.setFramerateLimit(60);
  sf::Clock clk;
  while (win.isOpen()) {
    sf::Event ev;
    while (win.pollEvent(ev)) {
      if (ev.type == sf::Event::Closed) win.close();
      if (ev.type != sf::Event::KeyPressed) continue;
      switch (ev.key.code) {
        case sf::Keyboard::Escape:
          win.close();
          break;
        case sf::Keyboard::Num1:
          app.scene = 1;
          app.rebuild();
          break;
        case sf::Keyboard::Num2:
          app.scene = 2;
          app.rebuild();
          break;
        case sf::Keyboard::M:
          app.prm.mode = Mode((int(app.prm.mode) + 1) % 3);
          app.rebuild();
          break;
        case sf::Keyboard::F:
          app.prm.fastHz = nextOption(kHzOptions, app.prm.fastHz);
          app.rebuild();
          break;
        case sf::Keyboard::LBracket:
          app.prm.restitution = std::max(0.0, app.prm.restitution - 0.1);
          app.rebuild();
          break;
        case sf::Keyboard::RBracket:
          app.prm.restitution = std::min(1.0, app.prm.restitution + 0.1);
          app.rebuild();
          break;
        case sf::Keyboard::Up:
          app.prm.projSpeed += 250;
          app.rebuild();
          break;
        case sf::Keyboard::Down:
          app.prm.projSpeed = std::max(250.0, app.prm.projSpeed - 250);
          app.rebuild();
          break;
        case sf::Keyboard::T:
          app.timeScale = nextOption(kScaleOptions, app.timeScale);
          break;
        case sf::Keyboard::Space:
          app.paused = !app.paused;
          break;
        case sf::Keyboard::N:
          if (app.paused) app.world.step();
          break;
        case sf::Keyboard::R:
          app.rebuild();
          break;
        case sf::Keyboard::C:
          app.showTrails = !app.showTrails;
          break;
        case sf::Keyboard::H:
          app.showHud = !app.showHud;
          break;
        default:
          break;
      }
    }
    app.advance(clk.restart().asSeconds());
    app.draw(win);
    win.display();
  }
  return 0;
}
