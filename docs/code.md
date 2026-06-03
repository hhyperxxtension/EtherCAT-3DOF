# Документация по коду системы управления

Справочник по исходному коду (`src/`) системы управления 2DOF-манипулятором с
компенсацией упругой деформации. Теория и формулы — в [algorithm.md](algorithm.md);
здесь — назначение модулей, классов, функций и их взаимосвязи.

## Содержание
1. [Архитектура и зависимости](#1-архитектура-и-зависимости)
2. [config.py — параметры](#2-configpy)
3. [kinematics.py — кинематика](#3-kinematicspy)
4. [joint_map.py — суставы ↔ энкодер](#4-joint_mappy)
5. [trajectory.py — планировщик](#5-trajectorypy)
6. [ethercat_csp.py — шинный слой CSP](#6-ethercat_csppy)
7. [gravity_model.py — модель G(q)](#7-gravity_modelpy)
8. [stiffness.py — жёсткость K_eff](#8-stiffnesspy)
9. [dynamics.py — динамическая модель](#9-dynamicspy)
10. [deflection.py — компенсация](#10-deflectionpy)
11. [controller.py — контроллер и CLI](#11-controllerpy)
12. [Файлы калибровки](#12-файлы-калибровки)
13. [Команды CLI](#13-команды-cli)
14. [Тесты](#14-тесты)
15. [Вспомогательные/устаревшие модули](#15-вспомогательныеустаревшие-модули)

---

## 1. Архитектура и зависимости

Поток данных рабочего цикла:
```
(x,z) → kinematics(IK) → q_target → controller._go → trajectory(план)
      → [компенсация] → joint_map(угол→отсчёты) → ethercat_csp(CSP) → привод
```

Граф зависимостей модулей (стрелка = «импортирует»):
```
controller ──► kinematics, joint_map, trajectory, ethercat_csp,
               gravity_model, stiffness, dynamics, deflection, config
deflection ──► gravity_model, stiffness, dynamics, joint_map, config
dynamics   ──► gravity_model, config
stiffness  ──► kinematics, config
ethercat_csp ► config, speed_test
trajectory ──► config            kinematics ──► config
joint_map  ──► config            gravity_model ► config
```

**Разделение «чистая математика» / «железо»:** `kinematics`, `joint_map`,
`trajectory`, `gravity_model`, `stiffness`, `dynamics`, `deflection` — чистые
(тестируются офлайн, без `pysoem`). `ethercat_csp` и часть `controller`
работают с шиной EtherCAT.

---

## 2. config.py

Единый файл констант. Импортируется почти всеми модулями. Основные группы:

| Группа | Ключи |
|--------|-------|
| Шина/цикл | `ECAT_ADAPTER_NAME`, `EC_CYCLE_TIME` (4 мс) |
| CiA 402 объекты | `CONTROLWORD`, `STATUSWORD`, `TARGET_VELOCITY`, `TORQUE_ACTUAL`(0x6077), `POSITION_ACTUAL`(0x6064), `TARGET_POSITION`(0x607A), `MAX_TORQUE`, `POS/NEG_TORQUE_LIMIT`(0x60E0/E1) |
| Состояния/команды | `CW_*`, `SW_*`, `MODE_CYCLIC_SYNC_POSITION`(8)/`VELOCITY`(9) |
| Лимит момента | `MAX_TORQUE_PERMILLE` |
| Геометрия | `LINK_LENGTHS`(мм), `ELBOW_CONFIG`, `JOINT_LIMITS`(рад) |
| Привод↔сустав | `GEAR_RATIO`, `ENCODER_COUNTS_PER_REV`, `JOINT_DIR`, `JOINT_ZERO_RAD` |
| Движение | `JOINT_VEL_MAX`, `JOINT_ACC_MAX` |
| Калибровки | `PROBE_*`, `GRAVCAL_*`, `STIFFCAL_*`, `STIFF_DROOP/SCALE_POSES_DEG`, `DYNCAL_*`, `Z_FLOOR*` |
| Наблюдатель | `OBS_LP_ALPHA`, `OBS_DELTA_MAX_RAD` |

Калиброванные значения стенда: `LINK_LENGTHS=(200,351)`, `GEAR_RATIO=(50,100)`,
`ENCODER_COUNTS_PER_REV=2**23`, `JOINT_DIR=(-1,-1)`, `JOINT_LIMITS` q0∈[−1°,181°],
q1∈±110°.

---

## 3. kinematics.py

Планарная 2DOF кинематика. Чистая математика. Длины звеньев в мм → все
декартовы величины в мм.

| Функция | Назначение |
|---------|-----------|
| `forward(q)` → (x,z) | прямая кинематика (положение кончика) |
| `elbow_position(q)` → (x,z) | положение локтя |
| `inverse(x,z, elbow)` → (q0,q1) | обратная кинематика; `elbow`∈{"up","down"}; бросает `Unreachable` |
| `inverse_both(x,z)` | обе ветви как dict |
| `reachable(x,z)` → bool | точка в кольце рабочей зоны |
| `within_joint_limits(q, limits)` → bool | проверка суставных пределов |
| `workspace_bounds()` → (r_min,r_max) | радиусы кольца |
| `jacobian(q)` → 2×2 | ∂(x,z)/∂(q0,q1) |
| `tip_offset(q, dq)` → (dx,dz) | смещение кончика для малого Δq (для вывода поправки в мм) |
| `Unreachable` | исключение (точка вне зоны) |

---

## 4. joint_map.py

Преобразование «угол сустава ↔ сырые отсчёты энкодера» (0x607A/0x6064) с учётом
редукции, направления и нуля. Чистая математика.

| Функция | Назначение |
|---------|-----------|
| `angle_to_counts(angle, joint, home_counts=0)` | угол → отсчёты |
| `counts_to_angle(counts, joint, home_counts=0)` | отсчёты → угол |
| `angles_to_counts(q, home_counts)` / `counts_to_angles(counts, home_counts)` | векторные версии |
| `counts_per_joint_rev(joint)` | отсчётов на оборот сустава = `GEAR_RATIO·CPR` |
| `counts_per_rad(joint)` | знаковый коэффициент отсчёт/рад (для поправок) |
| `deg_per_count(joint)` | угловое разрешение |
| `counts_in_int32(counts)` → bool | помещается ли в DINT 0x607A |

`home_counts` — отсчёт абсолютного энкодера в нулевой позе (захватывается при
`home`).

---

## 5. trajectory.py

Синхронизированный трапецеидальный планировщик (joint-space PTP). Чистая
математика. Отдаёт аналитические q, q̇, q̈ (без дифференцирования) — нужно
динамической модели.

| Сущность | Назначение |
|----------|-----------|
| `plan_ptp(q_start, q_target, v_max, a_max)` → `Trajectory` | планирование хода |
| `Trajectory.at(t)` / `at_full(t)` | (q) / (q, q̇, q̈) в момент t |
| `Trajectory.samples(dt)` / `samples_full(dt)` | дискретизация: список (t,q) / (t,q,q̇,q̈) |
| `Trajectory.duration` | полное время хода |
| `_JointProfile` | трапеция одного сустава: `at/vel/acc` |

Синхронизация: общее время = максимум по суставам; остальные замедляются.

---

## 6. ethercat_csp.py

Шинный слой Cyclic Sync Position (CiA 402, mode 8). Класс `CSPBus` владеет
EtherCAT-мастером и потоком реального времени. Запуск файла напрямую
(`python ethercat_csp.py`) — интерактивный инструмент калибровки джогом
(`rev`/`deg`/`cnt`, `home`).

**`CSPBus`:**

| Метод | Назначение |
|-------|-----------|
| `connect()` | поиск, маппинг PDO, DC-sync, **захват цели = текущая позиция**, выход в OP, включение CiA 402, старт RT-потока |
| `disconnect()` | freeze → отключение → Init → close |
| `actual_counts()` → list | текущие 0x6064 по приводам |
| `status()` → (snap, target, goal, faulted) | срез состояния |
| `play_trajectory(vectors, log_torque=False, corrector=None)` | поставить траекторию в очередь; опц. лог момента; опц. **живой корректор** (адаптивная δ) |
| `torque_log()` → list | помодульный момент за каждый цикл (для dyncal) |
| `is_busy()` / `freeze()` | играется ли траектория / стоп с удержанием |
| `set_goal_counts` / `move_by_counts` | джог |

**RT-цикл `_cycle_loop`** (каждые 4 мс): берёт точку из очереди → (если есть)
прибавляет смещение корректора → ограничивает шаг (`hard_max_step`) → обмен PDO
→ пишет статус/лог. **Защиты:** захват цели при включении, slew-ограничение,
лимит момента 0x60E0/0x60E1.

Свободные функции: `find_adapter`, `build_rx`/`parse_tx` (упаковка PDO),
`_setup_screen`/`_monitor` (экранный вывод калибровочного инструмента).

---

## 7. gravity_model.py

Модель статической нагрузки G(q), идентифицируется по данным (МНК), в ‰
момента мотора, по каждой оси отдельно.

**`GravityModel`:**

| Метод | Назначение |
|-------|-----------|
| `predict(q)` → (m0,m1) | предсказанная нагрузка по осям, ‰ |
| `fit(samples)` → rms | МНК по `[(q, τ_load), ...]`; базис `_basis` |
| `is_identified()` | коэффициенты заданы |
| `save(path)` / `load(path)` | `gravity_calib.json` |

Базис: ось0 `[cos q0, sin q0, cos(q0+q1), sin(q0+q1), 1]`, ось1
`[cos(q0+q1), sin(q0+q1), 1]`.

---

## 8. stiffness.py

Лумпированная жёсткость K_eff [‰/рад] и якобиан кончика по высоте.

| Сущность | Назначение |
|----------|-----------|
| `tip_jacobian(q)` → (J0,J1) | (∂z/∂q0, ∂z/∂q1) = (x_кончика, L2·cos(q0+q1)) |
| `StiffnessModel.deflection(q, load)` → (δ0,δ1) | δ_i = load_i/K_i (0 если K=None) |
| `StiffnessModel.fit(samples)` | МНК «tip-only» (устаревший один-шаг stiffcal) |
| `StiffnessModel.is_identified()` | **хотя бы одна** ось задана (частичная компенсация) |
| `StiffnessModel.save/load` | `stiffness_calib.json` |

K_eff заполняется двухэтапной калибровкой (см. `controller.fit_droop_stiffness`
+ `finalize_stiffness`). Ось с `K=None` → δ=0 (не компенсируется).

---

## 9. dynamics.py

Жёсткотельная динамическая модель — предсказывает момент **без груза**
(гравитация + инерция + Кориолис + трение), линейна по параметрам. Нужна
онлайн-наблюдателю, чтобы вычесть инерцию/трение из живого момента.

**`DynamicsModel`** (хранит ссылку на `GravityModel` + динамические коэффициенты):

| Метод | Назначение |
|-------|-----------|
| `predict(q,q̇,q̈)` → tuple | полный момент без груза = G(q) + динамика |
| `dyn_torque(q,q̇,q̈)` | только инерция+Кориолис+трение |
| `payload_residual(q,q̇,q̈,τ)` | τ − predict = вклад груза |
| `fit(samples)` → rms | МНК остатка `τ − G` по `[(q,q̇,q̈,τ), ...]` |
| `save/load` | `dynamics_calib.json` |

Базис `dyn_basis`: ось0 `[q̈0, q̈1, cos q1·(2q̈0+q̈1)−sin q1·(2q̇0q̇1+q̇1²), sgn q̇0, q̇0]`;
ось1 `[q̈0+q̈1, cos q1·q̈0+sin q1·q̇0², sgn q̇1, q̇1]`.

---

## 10. deflection.py

Компенсация деформации — два режима.

**`DeflectionCompensator`** (статика, упреждение от модели):

| Метод | Назначение |
|-------|-----------|
| `correction(q)` → δ | δ_i = G_i(q)/K_eff,i |
| `compensate(q_target)` → (q_cmd, δ) | q_cmd = q_target + δ |
| `load()` | из gravity+stiffness, иначе None |

**`LiveCompensator`** (адаптивный онлайн-наблюдатель, вызывается каждый цикл RT):

| Метод | Назначение |
|-------|-----------|
| `new_move(samples_full)` | привязать траекторию следующего хода (c_p и ramp сохраняются) |
| `__call__(status)` → offsets | по моменту: r=τ−predict, c_p=ΣJr/ΣJ² (НЧ-фильтр), δ=усиление·(G+c_p·J)/K, → смещение в отсчётах |
| `available()` | доступны gravity+stiffness+dynamics |
| `.last_cp`, `.last_delta` | для индикации (c_p, δ рад) |

Особенности: плавное включение (ramp `OBS_LP_ALPHA`, `ramp_s`), ограничение
`OBS_DELTA_MAX_RAD`, персистентность между ходами (c_p не сбрасывается).

---

## 11. controller.py

Главное приложение: класс `RobotController` (логика) + текстовый CLI (`main`).
Связывает кинематику, планировщик, joint_map, шину и компенсацию; содержит все
калибровочные процедуры.

### Жизненный цикл и состояние
`__init__` / `connect` / `disconnect`; `_load_models` (грузит gravity/stiffness/
dynamics/compensator); `comp_mode`∈{off, static, adaptive} (по умолчанию **off**);
`_static_ok`/`_adaptive_ok`/`_default_mode`.

### Home (нуль)
`set_home(persist)` / `load_home` / `home_plausible` / `clear_home` — захват и
персист отсчётов нуля (`home_calib.json`), с проверкой правдоподобности при
восстановлении.

### Состояние позы
`current_q()` → углы (из отсчётов), `current_xz()` → (x,z).

### Движение
| Метод | Назначение |
|-------|-----------|
| `move_to(x,z)` / `move_by(dx,dz)` | декартово абс./относ. |
| `move_joints(q)` / `move_joints_by(dq)` | суставное абс./относ. |
| `plan_move(x,z)` → q_target | валидация (зона/лимиты) |
| `_go(q_target)` | диспетчер режимов компенсации; в adaptive — план от **позиции звена** (current_q − last_δ) против рывка |
| `_compensate(q_target)` | статическая поправка эндпоинта |
| `correction_mm()` → (dx,dz,|.|) | поправка в мм на кончике (для вывода) |
| `stop()` / `wait()` | стоп с удержанием / ожидание конца |

### Калибровка
| Метод | Эксперимент |
|-------|-------------|
| `probe_joint`/`probe_pose` | bidirectional проба момента (де-фрикционирование) |
| `grav_grid`/`grav_calibrate` | автообход сетки → fit G(q) (`gravcal`) |
| `fit_droop_stiffness(samples)` | EXP1: Kp из просадки (NNLS) → `stiffness_phys.json` |
| `finalize_stiffness(scale)` | EXP2: масштаб s → K=s·Kp → `stiffness_calib.json` |
| `stiffcal_poses`/`measure_dm`/`fit_stiffness` | устаревший один-шаг stiffcal |
| `dyncal(...)` | автообход с разными ускорениями → fit DynamicsModel |
| `approach(q)` | floor-checked переезд (для калибровок) |

### Безопасность
`_pose_floor_ok`/`_pose_safe`/`_traj_floor_ok` — декартов «пол» (Z_FLOOR) для
кончика и локтя, в позах и на пути. Свободные хелперы: `_nnls` (неотрицательный
МНК), `_linspace`, `_deg`, `_monitor_move` (вывод хода + поправки в мм),
`_print_where`.

---

## 12. Файлы калибровки

Все в `src/`, в `.gitignore` (машинно-зависимые, создаются на железе):

| Файл | Создаёт | Содержимое |
|------|---------|-----------|
| `home_calib.json` | `home` | отсчёты нуля |
| `gravity_calib.json` | `gravcal` | коэффициенты G(q) |
| `gravity_samples.json` | `gravcal` | сырые (поза, нагрузка, трение) |
| `stiffness_phys.json` | `stiffdroop` | Kp [Н·мм/рад] + замеры просадки |
| `stiffness_calib.json` | `stiffscale` | итоговая K_eff [‰/рад] |
| `stiffness_samples.json` | `stiffcal` (legacy) | сырые (поза, dm, dz) |
| `dynamics_calib.json` | `dyncal` | динамические коэффициенты |
| `dynamics_samples.json` | `dyncal` | сырые (q,q̇,q̈,τ) |

Порядок калибровки: `home` → `gravcal` → `stiffdroop` → `stiffscale` → `dyncal`.

---

## 13. Команды CLI

Запуск: `python controller.py`. Все расстояния — мм, углы — градусы.

| Команда | Действие |
|---------|----------|
| `home` / `rehome` / `clearhome` | задать/переопределить/удалить нуль |
| `where` | текущие углы, (x,z), отсчёты |
| `move x z` / `rmove dx dz` | декартово абс./относ. |
| `jmove q0 q1` / `jrmove d0 d1` | суставное абс./относ. |
| `elbow up\|down` | ветвь IK |
| `speed v a` | лимиты скорости/ускорения сустава |
| `probe [sweep speed]` | замер нагрузки+трения в текущей позе |
| `gravcal [n0 n1]` | калибровка G(q) |
| `stiffdroop` | EXP1 жёсткости (просадка) |
| `stiffscale` | EXP2 жёсткости (масштаб) → финал K_eff |
| `stiffcal [n]` | устаревшая один-шаг калибровка |
| `dyncal` | идентификация динамики |
| `comp [off\|static\|adaptive]` | режим компенсации/статус |
| `stop` | стоп с удержанием |
| `q` | выход (отключение приводов) |

---

## 14. Тесты

`tests/*.py` — офлайн, запуск `python tests/<name>.py` (без pytest):

| Файл | Покрывает |
|------|-----------|
| `test_kinematics.py` | FK/IK, зона, лимиты |
| `test_trajectory.py` | трапеция, синхронизация |
| `test_joint_map.py` | угол↔отсчёты, направление, INT32 |
| `test_gravity_model.py` | fit/predict/save G(q) |
| `test_stiffness.py` | fit/deflection/якобиан |
| `test_dynamics.py` | производные траектории, fit, остаток груза |

---

## 15. Вспомогательные/устаревшие модули

- `speed_test.py` — прототип скоростного управления; экспортирует
  `VELOCITY_FACTOR` (используется `ethercat_csp` для масштаба 0x6080/скорости).
- `csv_multi.py` — многоприводное управление по скорости (CSV, mode 9);
  ранний рабочий контроллер для джога, заменён на CSP-слой для позиционирования.
- `docs/make_figures.py` — генерация рисунков для `algorithm.md`/`.tex`.

> Полное теоретическое описание (формулы, выводы, блок-схема) — в
> [algorithm.md](algorithm.md) и [algorithm.tex](algorithm.tex).
