# Guida al Tuning — armor_tracker & rm_trajectory

Questa guida descrive tutti i parametri regolabili dei nodi `armor_tracker` e `rm_trajectory`, spiega il loro effetto sul comportamento del sistema e fornisce indicazioni pratiche per il tuning.

---

## Indice

1. [Strumenti di debug](#1-strumenti-di-debug)
2. [armor_tracker — Parametri ROS](#2-armor_tracker--parametri-ros)
3. [armor_tracker — EKF Process Noise (Q)](#3-armor_tracker--ekf-process-noise-q)
4. [armor_tracker — EKF Measurement Noise (R)](#4-armor_tracker--ekf-measurement-noise-r)
5. [armor_tracker — Covarianza Iniziale (P0)](#5-armor_tracker--covarianza-iniziale-p0)
6. [armor_tracker — Costanti Hardcoded](#6-armor_tracker--costanti-hardcoded)
7. [rm_trajectory — Parametri ROS](#7-rm_trajectory--parametri-ros)
8. [rm_trajectory — Costanti Hardcoded](#8-rm_trajectory--costanti-hardcoded)
9. [Workflow di Tuning Consigliato](#9-workflow-di-tuning-consigliato)
10. [Problemi Comuni e Soluzioni](#10-problemi-comuni-e-soluzioni)

---

## 1. Strumenti di debug

Prima di iniziare il tuning, familiarizzare con i topic di debug disponibili.

### Topic pubblicati da armor_tracker

| Topic | Tipo | Contenuto |
|---|---|---|
| `/tracker/info` | `TrackerInfo` | `position_diff`, `yaw_diff`, posizione misurata, yaw misurato |
| `/tracker/target` | `Target` | Stato EKF completo: posizione, velocita, yaw, v_yaw, raggi, dz |
| `/tracker/marker` | `MarkerArray` | Visualizzazione RViz: armor, robot center, velocita |

### Topic pubblicati da rm_trajectory

| Topic | Tipo | Contenuto |
|---|---|---|
| `/tracker/cmd_gimbal` | `GimbalCmd` | pitch, yaw, distanza, fire_cmd |
| `/cmd_vel` | `Twist` | angular.x = fire_cmd, angular.y/z = angoli in gradi |
| `/trajectory/marker` | `Marker` | Punto di impatto predetto (sfera in RViz) |

### Comandi utili per monitoraggio

```bash
# Monitorare lo stato del tracker in tempo reale
ros2 topic echo /tracker/target

# Monitorare i comandi gimbal
ros2 topic echo /tracker/cmd_gimbal

# Monitorare le info di debug (position_diff e yaw_diff)
ros2 topic echo /tracker/info

# Visualizzare i parametri correnti
ros2 param list /armor_tracker_node
ros2 param get /armor_tracker_node ekf.sigma2_q_xyz

# Modificare un parametro a runtime (se il nodo supporta dynamic reconfigure)
ros2 param set /armor_tracker_node ekf.sigma2_q_xyz 8.0
```

---

## 2. armor_tracker — Parametri ROS

Tutti dichiarati in `tracker_node.cpp` e modificabili da launch file o YAML.

### `max_armor_distance` — Distanza massima armor

| | |
|---|---|
| **Default** | `10.0` m |
| **Tipo** | double |
| **Effetto** | Armature con distanza nel piano XOY superiore a questo valore vengono scartate prima del tracking. |
| **Tuning** | Ridurre a `6.0–8.0` se l'arena e piccola o se le detection lontane sono troppo rumorose. Aumentare se il robot deve ingaggiare a lungo raggio (sentry). |

### `light_ratio` — Fattore di correzione light-bar PnP

| | |
|---|---|
| **Default** | `0.85` |
| **Tipo** | double |
| **Effetto** | Scala la larghezza della bounding box per approssimare la separazione reale dei light-bar. Riduce il bias sistematico nella stima PnP (yaw e posizione). |
| **Tuning** | Monitorare `/tracker/info` → `yaw_diff`. Se `yaw_diff` ha un bias costante, aggiustare `light_ratio` in incrementi di `0.02`. Valore tipico: `0.80–0.90`. A distanza ~5 m senza correzione il bias e ~8 cm. |

### `tracker.max_match_distance` — Soglia di matching posizionale

| | |
|---|---|
| **Default** | `0.15` m |
| **Tipo** | double |
| **Effetto** | Distanza massima tra la posizione predetta dall'EKF e la misura per considerare un match valido. |
| **Tuning** | Se il tracker perde frequentemente il target durante manovre brusche, aumentare a `0.20–0.25`. Se associa armature sbagliate (target diversi), ridurre a `0.10`. |

### `tracker.max_match_yaw_diff` — Soglia di matching angolare

| | |
|---|---|
| **Default** | `1.0` rad |
| **Tipo** | double |
| **Effetto** | Differenza massima di yaw per il matching. Viene **sovrascritta dinamicamente** in base al tipo di robot: `pi - 0.3` (2 armature), `2pi/3 - 0.3` (3 armature), `pi/2 - 0.3` (4 armature). |
| **Tuning** | Usato come fallback. Generalmente non necessita di modifica. |

### `tracker.tracking_thres` — Soglia di conferma tracking

| | |
|---|---|
| **Default** | `5` |
| **Tipo** | int |
| **Effetto** | Numero di match consecutivi necessari per passare da DETECTING a TRACKING. |
| **Tuning** | Ridurre a `3` per risposta piu rapida (rischio: falsi positivi). Aumentare a `7–10` per robustezza in ambienti con molte false detection. |

### `tracker.lost_time_thres` — Tempo prima di perdere il target

| | |
|---|---|
| **Default** | `0.3` s |
| **Tipo** | double |
| **Effetto** | Tempo massimo in stato TEMP_LOST prima di passare a LOST. Convertito in frame: `lost_thres = ceil(lost_time_thres / dt)`. |
| **Tuning** | Aumentare a `0.5–0.8` se il target viene temporaneamente occluso (es. dietro ostacoli). Ridurre a `0.15–0.2` se servono transizioni rapide tra target diversi. |

### `ekf.yaw_damping_alpha` — Smorzamento velocita angolare

| | |
|---|---|
| **Default** | `0.98` |
| **Tipo** | double |
| **Effetto** | Coefficiente di damping per la velocita angolare nel modello di processo. `v_yaw_damped = v_yaw * alpha^(dt/T)` con T = 1/30 s. Alpha = 0.98 a 30 Hz da un tempo di dimezzamento ~1.1 s. |
| **Tuning** | Aumentare (verso `0.99–1.0`) per robot che mantengono lo spin a lungo (sentry spin). Ridurre (verso `0.95`) se il v_yaw stimato non decresce abbastanza velocemente quando il robot smette di girare. |

---

## 3. armor_tracker — EKF Process Noise (Q)

La matrice Q controlla quanto l'EKF si "fida" del modello di processo rispetto alle misure. Valori Q piu alti rendono il filtro piu reattivo ma piu rumoroso.

### `ekf.sigma2_q_xyz` — Rumore di processo per posizione XYZ

| | |
|---|---|
| **Default** | `5.0` |
| **Tipo** | double |
| **Effetto** | Varianza del rumore sugli stati di posizione (xc, yc, za) e velocita (v_xc, v_yc, v_za). Applicata come: `q_pos = (dt^4/4) * sigma2_q_xyz`, `q_vel = dt^2 * sigma2_q_xyz`. |
| **Tuning** | |

| Scenario | Azione | Valore suggerito |
|---|---|---|
| Target lenti/stazionari | Ridurre | `2.0–3.0` |
| Target agili con manovre brusche | Aumentare | `8.0–15.0` |
| Tracking smooth ma lento a reagire | Aumentare | `10.0+` |
| Stima oscillante/rumorosa | Ridurre | `2.0–4.0` |

### `ekf.sigma2_q_yaw` — Rumore di processo per yaw

| | |
|---|---|
| **Default** | `10.0` |
| **Tipo** | double |
| **Effetto** | Varianza del rumore sugli stati di yaw e v_yaw. Controlla quanto rapidamente il filtro si adatta a cambiamenti nella rotazione. |
| **Tuning** | |

| Scenario | Azione | Valore suggerito |
|---|---|---|
| Robot nemici con spin costante | Ridurre | `5.0–8.0` |
| Spin che cambia rapidamente | Aumentare | `15.0–25.0` |
| Armor jump detection troppo sensibile | Ridurre | `5.0` |
| Armor jump detection troppo lento | Aumentare | `20.0+` |

### `ekf.sigma2_q_r` — Rumore di processo per raggio

| | |
|---|---|
| **Default** | `2.0` |
| **Tipo** | double |
| **Effetto** | Varianza del rumore sullo stato `r` (distanza dell'armatura dal centro del robot). |
| **Tuning** | Generalmente stabile. Aumentare a `3.0–5.0` solo se il raggio stimato converge troppo lentamente dopo un cambio target. Ridurre a `0.5–1.0` se il raggio oscilla troppo (il raggio fisico e costante). |

---

## 4. armor_tracker — EKF Measurement Noise (R)

La matrice R controlla quanto l'EKF si "fida" delle misure. Modello dipendente dalla distanza: `R = (base + slope * distance)^2`.

### `ekf.r_xyz_base` / `ekf.r_xyz_slope` — Rumore misura posizione

| | |
|---|---|
| **Default** | base = `0.005`, slope = `0.03` |
| **Tipo** | double |
| **Effetto** | A distanza `d`: `sigma_xyz = 0.005 + 0.03 * d`. Es: a 3 m → sigma = 0.095, varianza = 0.009; a 6 m → sigma = 0.185, varianza = 0.034. |
| **Tuning** | |

| Scenario | Azione |
|---|---|
| PnP instabile a lungo raggio | Aumentare `r_xyz_slope` a `0.05` |
| Tracking troppo smooth (non segue le manovre) | Ridurre `r_xyz_base` a `0.002` |
| Posizione stimata rumorosa a breve distanza | Aumentare `r_xyz_base` a `0.01` |

### `ekf.r_yaw_base` / `ekf.r_yaw_slope` — Rumore misura yaw

| | |
|---|---|
| **Default** | base = `0.015`, slope = `0.002` |
| **Tipo** | double |
| **Effetto** | A distanza `d`: `sigma_yaw = 0.015 + 0.002 * d`. Es: a 3 m → sigma = 0.021 rad (~1.2 deg). |
| **Tuning** | Se la stima yaw oscilla, aumentare `r_yaw_base`. Se l'EKF non segue cambi rapidi di yaw, ridurre entrambi. |

---

## 5. armor_tracker — Covarianza Iniziale (P0)

Hardcoded in `tracker_node.cpp` (linee 130-132). Modifica richiede ricompilazione.

```cpp
p0.diagonal() << 0.1,  1.0,  0.1,  1.0,  0.1,  0.2,  0.1,  3.0,  0.003;
//                xc   v_xc  yc   v_yc  za   v_za  yaw  v_yaw   r
```

| Stato | Varianza Iniziale | Note |
|---|---|---|
| xc, yc | 0.1 | Incertezza posizione iniziale |
| v_xc, v_yc | 1.0 | Incertezza velocita iniziale (alta = non sappiamo la velocita) |
| za | 0.1 | Incertezza altezza |
| v_za | 0.2 | Incertezza velocita verticale |
| yaw | 0.1 | Incertezza yaw iniziale |
| v_yaw | 3.0 | Incertezza v_yaw molto alta (non sappiamo se gira) |
| r | 0.003 | Incertezza raggio molto bassa (dimensione fisica nota) |

**Suggerimento**: Se dopo l'inizializzazione il tracker impiega troppo tempo a convergere su v_yaw, aumentare P0[7] a `5.0`. Se il raggio oscilla troppo all'inizio, ridurre P0[8] a `0.001`.

---

## 6. armor_tracker — Costanti Hardcoded

Queste richiedono ricompilazione (`colcon build --packages-select armor_tracker`).

### Limiti di saturazione stato (tracker.cpp:116-131)

| Stato | Min | Max | Note |
|---|---|---|---|
| r (raggio) | 0.12 m | 0.4 m | Previene divergenza |
| v_yaw | -15.0 rad/s | +15.0 rad/s | Clamp per velocita angolari fisicamente impossibili |

### Raggio iniziale (tracker.cpp:164)

| Costante | Valore | Note |
|---|---|---|
| r iniziale | 0.26 m | Distanza iniziale armor-centro. Modificare se i robot hanno geometria diversa |

### Filtro altezza (tracker_node.cpp:325)

| Costante | Valore | Note |
|---|---|---|
| z max | 1.2 m | Armature con `|z| > 1.2` vengono scartate |

### Classificazione armor (tracker_node.cpp:247)

| Costante | Valore | Note |
|---|---|---|
| Soglia LARGE | width/height > 3.0 | Classificazione fallback (preferito il class_id dal detector) |

### Limiti dt (tracker_node.cpp:349)

| Costante | Min | Max | Note |
|---|---|---|---|
| dt | 0.01 s | 0.10 s | Clamp del delta-t per stabilita numerica |

### Dimensioni armor PnP (pnp_solver.hpp:36-39)

| Tipo | Larghezza | Altezza |
|---|---|---|
| SMALL | 135 mm | 55 mm |
| LARGE | 225 mm | 55 mm |

**Importante**: Queste dimensioni devono corrispondere esattamente alle armature fisiche. Se il PnP ha un bias sistematico nella distanza, verificare questi valori con un metro.

---

## 7. rm_trajectory — Parametri ROS

Tutti dichiarati in `trajectory_solver.cpp` e modificabili da launch file.

### `bullet_speed` — Velocita del proiettile

| | |
|---|---|
| **Default** | `25.0` m/s |
| **Tipo** | double |
| **Effetto** | Velocita iniziale del proiettile. Influenza direttamente il tempo di volo e la compensazione di gravita. |
| **Tuning** | **Misurare sperimentalmente** con cronografo. Variazioni di 1 m/s hanno impatto significativo a lunga distanza. Aggiornare se si cambia tipo di proiettile o pressione. |

### `gravity` — Accelerazione gravitazionale

| | |
|---|---|
| **Default** | `9.8` m/s^2 |
| **Tipo** | double |
| **Tuning** | Non modificare a meno di compensare bias sistematici nel pitch. Se i colpi cadono costantemente corti, provare `9.9–10.0` come compensazione empirica. |

### `k` — Coefficiente di drag

| | |
|---|---|
| **Default** | `0.01` |
| **Tipo** | double |
| **Effetto** | Modella la resistenza dell'aria. Velocita effettiva: `v_eff = v0 * (1 - e^(-k*t)) / (k*t)`. |
| **Tuning** | Determinare sperimentalmente sparando a target fissi a diverse distanze (3, 5, 7 m). Se i colpi cadono corti a lungo raggio rispetto al modello, aumentare `k`. Valori tipici: `0.005–0.03`. |

### `time_bias` — Compensazione latenza iniziale

| | |
|---|---|
| **Default** | `0.08` s |
| **Tipo** | double |
| **Effetto** | Offset temporale aggiunto al tempo di volo per compensare la latenza della pipeline (camera → detection → tracking → solver). Usato come valore iniziale per l'EMA adattivo. |
| **Tuning** | Misurare la latenza reale con: `ros2 topic echo /tracker/cmd_gimbal --field header.stamp` e confrontare con l'orologio. Valori tipici su Jetson: `0.05–0.15` s. |

### `time_bias_alpha` — Fattore smoothing latenza

| | |
|---|---|
| **Default** | `0.35` |
| **Tipo** | double |
| **Effetto** | Peso per l'aggiornamento EMA della latenza: `bias_new = alpha * measured + (1-alpha) * bias_old`. Alpha piu alto = reazione piu rapida a variazioni di latenza. |
| **Tuning** | Ridurre a `0.1–0.2` per latenza piu stabile. Aumentare a `0.5` se la latenza varia molto (es. carico GPU variabile). |

### `min_fire_dist` / `max_fire_dist` — Range di ingaggio

| | |
|---|---|
| **Default** | min = `0.5` m, max = `10.0` m |
| **Tipo** | double |
| **Effetto** | Il comando di fuoco viene inibito fuori da questo range. |
| **Tuning** | Impostare `max_fire_dist` in base alla precisione reale del robot. Se i colpi oltre 6 m non centrano, ridurre a `6.0`. |

### `max_spin_rate` — Soglia spin per target lenti

| | |
|---|---|
| **Default** | `4.0` rad/s |
| **Tipo** | double |
| **Effetto** | Nel modo "target lento" (`|v_yaw| < min_spin_rate_for_predictor`), il fuoco e bloccato se `|v_yaw| > max_spin_rate`. |
| **Tuning** | Aumentare se si vuole sparare a target che girano moderatamente. Ridurre se si spreca munizioni su target in spin. |

### `min_spin_rate_for_predictor` — Soglia attivazione predittore

| | |
|---|---|
| **Default** | `1.5` rad/s |
| **Tipo** | double |
| **Effetto** | Soglia che attiva il passaggio dal gate semplice al gate predittivo (finestra angolare). Sotto questa soglia si usa il controllo `max_spin_rate`. Sopra, si usa il predittore di finestra armor. |
| **Tuning** | Ridurre a `0.8–1.0` per attivare il predittore prima (utile se il robot nemico ha spin variabile). Aumentare a `2.0` se il predittore causa falsi fire_cmd a bassi spin rate. |

### `angular_window` — Finestra angolare di fuoco

| | |
|---|---|
| **Default** | `0.09` rad (~5.2 deg) |
| **Tipo** | double |
| **Effetto** | Semi-larghezza della finestra angolare entro cui il fire_cmd viene attivato. La finestra effettiva e espansa dall'incertezza del yaw EKF: `effective_window = angular_window + sigma_vyaw * t_flight`. |
| **Tuning** | |

| Scenario | Azione | Valore |
|---|---|---|
| Colpi mancati su target in spin | Aumentare | `0.12–0.15` |
| Troppi colpi fuori bersaglio | Ridurre | `0.05–0.07` |
| Proiettili grandi (17 mm) | Aumentare | `0.12` |
| Proiettili piccoli (10 mm) con alta velocita | Ridurre | `0.06` |

---

## 8. rm_trajectory — Costanti Hardcoded

Richiedono ricompilazione (`colcon build --packages-select rm_trajectory`).

| Costante | Valore | File:Linea | Descrizione |
|---|---|---|---|
| `max_iter` | 10 | trajectory_solver.cpp:56 | Iterazioni massime del solver balistico |
| Convergence tolerance | 1e-4 s | trajectory_solver.cpp:82 | Tolleranza convergenza tempo di volo |
| Pitch clamp | +/- 1.4 rad | trajectory_solver.cpp:64-65 | Limiti angolo di elevazione (~80 deg) |
| Latency validity | 0.0–0.5 s | trajectory_solver.cpp:108 | Range accettabile per misura latenza |

---

## 9. Workflow di Tuning Consigliato

### Fase 1: Verifica PnP (armatura singola, statica)

1. Posizionare un'armatura a distanza nota (es. 3 m)
2. Verificare che `/tracker/target` → `position` corrisponda alla distanza reale
3. Se c'e bias:
   - Distanza sbagliata → Verificare dimensioni armor in `pnp_solver.hpp`
   - Yaw con offset costante → Regolare `light_ratio`
4. Ripetere a 1 m, 3 m, 5 m, 7 m

### Fase 2: Tuning EKF (armatura in movimento)

1. Muovere l'armatura lentamente e monitorare `/tracker/info`:
   - `position_diff` alto → Aumentare `sigma2_q_xyz` o ridurre `r_xyz_base`
   - Posizione stimata rumorosa → Ridurre `sigma2_q_xyz` o aumentare `r_xyz_base`
2. Ruotare l'armatura e monitorare `yaw_diff`:
   - `yaw_diff` costantemente alto → Aumentare `sigma2_q_yaw`
   - Yaw stimato rumoroso → Ridurre `sigma2_q_yaw`
3. Testare con robot in spin:
   - Se `v_yaw` converge lentamente → Aumentare `sigma2_q_yaw` e/o P0[7]
   - Se `v_yaw` oscilla troppo → Aumentare `yaw_damping_alpha`

### Fase 3: Tuning Balistico (sparare a target fisso)

1. Sparare a target fermo a 3 m
2. Se i colpi cadono **sotto**: aumentare `bullet_speed` o ridurre `gravity`
3. Se i colpi cadono **sopra**: ridurre `bullet_speed` o aumentare `gravity`
4. Ripetere a 5 m e 7 m:
   - Se il bias cresce con la distanza → regolare `k` (drag)
5. Verificare che il solver converga: controllare log per warning di non-convergenza

### Fase 4: Tuning Latenza (target in movimento)

1. Sparare a target che si muove lateralmente a velocita costante
2. Se i colpi cadono **dietro** il target (ritardo): aumentare `time_bias`
3. Se i colpi cadono **davanti** (anticipa troppo): ridurre `time_bias`
4. Regolare `time_bias_alpha` in base alla stabilita della correzione

### Fase 5: Tuning Fire Gate (target in spin)

1. Testare su robot nemico in spin a distanza media (3-5 m)
2. Se non spara mai → Aumentare `angular_window` e/o `max_spin_rate`
3. Se spara troppo (colpi a vuoto) → Ridurre `angular_window`
4. Se spara solo quando il target si ferma → Ridurre `min_spin_rate_for_predictor`

### Fase 6: Tuning Tracker State Machine

1. Osservare le transizioni di stato con `ros2 topic echo /tracker/target --field tracker_state`
2. Se il tracker si attacca troppo lentamente → Ridurre `tracking_thres`
3. Se il tracker perde il target troppo facilmente → Aumentare `lost_time_thres`
4. Se il tracker rimane attaccato a target sbagliati → Ridurre `lost_time_thres`

---

## 10. Problemi Comuni e Soluzioni

### Il tracker perde il target durante lo spin

- Aumentare `tracker.max_match_distance` (es. `0.20`)
- Aumentare `tracker.lost_time_thres` (es. `0.5`)
- Verificare che `ekf.sigma2_q_yaw` sia abbastanza alto (es. `15.0`)

### La posizione stimata oscilla troppo

- Ridurre `ekf.sigma2_q_xyz` (es. `3.0`)
- Aumentare `ekf.r_xyz_base` (es. `0.01`)
- Verificare che il dt sia stabile (frame rate costante)

### Il PnP da valori incoerenti a lunga distanza

- Aumentare `ekf.r_xyz_slope` (es. `0.05`) per dare meno peso alle misure lontane
- Ridurre `max_armor_distance` per scartare detection troppo lontane

### Il fire_cmd non si attiva mai

- Verificare range: `ros2 topic echo /tracker/cmd_gimbal --field distance`
- Verificare che `min_fire_dist` / `max_fire_dist` siano corretti
- Aumentare `angular_window` (es. `0.12`)
- Aumentare `max_spin_rate` se il target gira

### Colpi sistematicamente fuori bersaglio

- Misurare `bullet_speed` con cronografo e aggiornare il parametro
- Verificare il `time_bias` analizzando la latenza effettiva
- Se il bias cambia con la distanza, regolare `k` (drag)
- Verificare la calibrazione della camera (intrinsics)

### Armor jump non rilevati correttamente

- Verificare che `armors_num` nel messaggio Target sia corretto
- Se troppi falsi jump → Ridurre `sigma2_q_yaw`
- Se jump non rilevati → Aumentare `sigma2_q_yaw`, verificare matching thresholds

---

## Appendice: Esempio Configurazione Launch File

```python
# Configurazione esempio per sentry
ComposableNode(
    package='armor_tracker',
    plugin='rm_auto_aim::ArmorTrackerNode',
    name='armor_tracker_node',
    parameters=[{
        'max_armor_distance': 8.0,
        'light_ratio': 0.85,
        'tracker.max_match_distance': 0.18,
        'tracker.max_match_yaw_diff': 1.0,
        'tracker.tracking_thres': 5,
        'tracker.lost_time_thres': 0.4,
        'target_frame': 'odom',
        'ekf.yaw_damping_alpha': 0.98,
        'ekf.sigma2_q_xyz': 5.0,
        'ekf.sigma2_q_yaw': 10.0,
        'ekf.sigma2_q_r': 2.0,
        'ekf.r_xyz_base': 0.005,
        'ekf.r_xyz_slope': 0.03,
        'ekf.r_yaw_base': 0.015,
        'ekf.r_yaw_slope': 0.002,
    }],
)

ComposableNode(
    package='rm_trajectory',
    plugin='rm_auto_aim::TrajectorySolverNode',
    name='trajectory_solver_node',
    parameters=[{
        'bullet_speed': 25.0,
        'gravity': 9.8,
        'k': 0.01,
        'time_bias': 0.08,
        'time_bias_alpha': 0.35,
        'min_fire_dist': 0.5,
        'max_fire_dist': 8.0,
        'max_spin_rate': 4.0,
        'min_spin_rate_for_predictor': 1.5,
        'angular_window': 0.09,
    }],
)
```
