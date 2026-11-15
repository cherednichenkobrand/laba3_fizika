#pragma once
#include <cstdint>
#include <deque>
#include <vector>

#include "config.hpp"
#include "vec2.hpp"

// Три режима расчёта движения и столкновений
enum class Mode {
  Discrete = 0,   // всё на 60 Гц, проверка пересечения только в конце шага
  CCD = 1,        // всё на 60 Гц, поиск момента столкновения t_c внутри шага
  MultiRate = 2,  // медленные 60 Гц (дискретно), быстрые fastHz (дискретно,
                  // sub-stepping)
};
const char* modeName(Mode m);

// Динамический или статический круг
struct Body {
  Vec2 p, v;   // положение и скорость
  Vec2 prevP;  // положение на начало «медленного» шага — для интерполяции
               // отрисовки
  double r = 10.0;
  double invMass = 0.0;  // 0 => неподвижный
  bool fast = false;     // снаряд?
  bool active = true;    // снаряд ещё не запущен => false

  // Управление запуском снарядов
  double timer = 0.0;   // время до (пере)запуска
  double period = 1.0;  // период перезапуска
  Vec2 launchPos, launchVel;

  // Для визуализации
  std::deque<Vec2> trail;  // положения в конце каждого шага физики
  std::deque<Vec2> hits;   // точки контакта со стенкой / препятствиями
};

// Неподвижный прямоугольник без вращения (AABB): стены арены и тонкая стенка
struct Box {
  Vec2 mn, mx;
  bool thin = false;  // «тонкая стенка» — на ней считаем туннелирование
};

struct Params {
  Mode mode = Mode::Discrete;
  double slowHz = cfg::kSlowHz;
  double fastHz = cfg::kFastHz;
  double restitution = cfg::kRestitution;
  double gravity = cfg::kGravity;
  double projSpeed = cfg::kProjectileSpeed;
  double projRadius = cfg::kProjectileRadius;
  double wallThickness = cfg::kWallThickness;
  int staticCount = cfg::kStaticCount;
  int slowCount = cfg::kSlowCount;
  int fastCount = cfg::kFastCount;
  double restThreshold = cfg::kRestThreshold;
  int solverIters = cfg::kSolverIterations;
  int maxToiEvents = cfg::kMaxToiEvents;
  double slop = 0.5;     // допустимое перекрытие, px
  double percent = 0.8;  // доля коррекции перекрытия за итерацию
  bool trails = true;
  unsigned seed = cfg::kSeed;
  double scene1StartX = -1.0;  // <0 => значение по умолчанию
};

// Счётчики, которые вызывающий код обнуляет раз в кадр.
struct StepStats {
  long long slowSteps = 0;
  long long fastSteps = 0;  // шагов, в которых интегрировались снаряды
  long long toiEvents = 0;
  long long toiCapHits = 0;  // сколько раз упёрлись в лимит событий CCD
  long long pairTests = 0;
};

class World {
 public:
  Params prm;
  int scene = 1;
  std::vector<Body> dyn;   // динамические: обычные + снаряды
  std::vector<Body> pegs;  // неподвижные круги
  std::vector<Box> boxes;  // неподвижные AABB
  double simTime = 0.0;
  long long stepIndex = 0;
  long long tunnelCount = 0;  // снаряд оказался по другую сторону тонкой стенки
  long long wallHits = 0;     // снаряд корректно ударился о тонкую стенку
  double lastThinPen =
      0.0;  // глубина перекрытия со стенкой в момент обнаружения удара
  StepStats stats;

  static constexpr double kWidth = 1280.0;
  static constexpr double kHeight = 720.0;

  void build(int sceneId, const Params& p);
  void step();  // один «медленный» шаг dt = 1/slowHz
  double dtSlow() const { return 1.0 / prm.slowHz; }
  int fastSubsteps() const;
  uint64_t stateHash() const;
  double maxPenetration() const;

 private:
  void updateLaunchers(double dt);
  void integrate(const std::vector<int>& idx, double dt, bool posToo);
  void solveDiscrete(const std::vector<int>& A, int iters);
  // A — группа, которую шагаем; B — тела, с которыми A может столкнуться (сами
  // B между собой не проверяются)
  void solveDiscrete2(const std::vector<int>& A, const std::vector<int>& B,
                      int iters);
  void stepCCD(const std::vector<int>& active, double dt);
  void advance(const std::vector<int>& idx, double t);
  bool resolve(Body& a, Body* b, const Vec2& n, double pen, bool positional);
  void afterFastStep(const std::vector<int>& fastIdx,
                     const std::vector<Vec2>& before);
};

// Геометрические функции (вынесены для тестов и отчёта)
bool overlapCircleCircle(const Vec2& pa, double ra, const Vec2& pb, double rb,
                         Vec2& n, double& pen);
bool overlapCircleBox(const Vec2& c, double r, const Box& b, Vec2& n,
                      double& pen);
// d = pA - pB, w = vA - vB, R = rA + rB. Возвращает момент первого касания t in
// [0, tmax]
bool toiCircleCircle(const Vec2& d, const Vec2& w, double R, double tmax,
                     double& t);
// Круг (c, v, r) против неподвижного AABB: момент касания и нормаль (от коробки
// к кругу)
bool toiCircleBox(const Vec2& c, const Vec2& v, double r, const Box& b,
                  double tmax, double& t, Vec2& n);
