# Ricerca tecnica su ARCC 2026 e sulle differenze dei team cinesi nelle stagioni recenti

## Contesto e perimetro

La premessa importante è questa: **ARCC 2026 non è più semplicemente “RMNA uguale a RMUL tradotto”**. ARC Robotics dichiara ufficialmente che ARCC 2026 ha propri manuali, che questi manuali sono la *source of truth* per la competizione, e che la competizione ha ora maggiore libertà rispetto al passato nel definire Participant, Robot Building e Game rules; allo stesso tempo, il sito ARCC conferma che il Robot Building Specifications rimanda ancora ai manuali RoboMaster come base tecnica di riferimento. Inoltre ARCC 2026 è strutturata su **1v1, 3v3 ed Engineer Challenge**, quindi per il confronto ho volutamente escluso tutto ciò che è tipicamente marginale per questo perimetro, come runes/energy mechanism o moduli pensati solo per scenari RMUC 7v7 che non si trasferiscono bene in ARCC. citeturn30search2turn30search1turn30search3turn30search0turn33view2

C’è però un limite metodologico che va detto in modo esplicito: **non ho visibilità diretta della vostra repository/pipeline privata in questa sessione**, quindi non posso fare un diff riga-per-riga contro “la vostra pipeline”. Il confronto che segue usa quindi come baseline una pipeline ARC/RM tipica, cioè: **detector 2D → PnP/frame-wise pose → filtro di tracking → balistica → regola di fuoco → stack di navigazione separato**, che è sostanzialmente il tipo di architettura reso popolare nell’ecosistema `rm_vision`/`rm_auto_aim`. citeturn20view1turn29search1turn29search8turn29search9

La conclusione anticipata, però, è già chiara: **i team cinesi forti 2024–2026 non stanno vincendo perché “hanno solo un detector migliore”**. La differenza grossa è un’altra: stanno ottimizzando **la catena temporale completa**, stanno rendendo **coerenti tra loro perception, target-state estimation, fire decision e controllo**, e nella navigation stanno passando da stack “SLAM + planner” generici a pipeline **sim2real, timestamp-consistent, TF-consistent, con relocalization e local planning pensati per dinamiche aggressive**. È lì che oggi c’è il vero gap. citeturn17search12turn21search13turn17search19turn17search9turn23search3

## Sintesi esecutiva

Se devo dirlo in una frase: **per ARCC 2026 io non copierei dai cinesi il “pezzo singolo”, copierei la filosofia di integrazione**. In particolare, le pratiche che emergono con più forza nelle stagioni 2025–2026 sono queste: misurazione esplicita delle latenze end-to-end; modello di tiro che ragiona sulla **traiettoria futura** del bersaglio e non sul solo frame corrente; logiche di fuoco **asimmetriche** per target rotanti; CV modulare con backend multipli (`opencv`, `openvino`, `tensorrt`) e deployment su acceleratori dedicati; navigazione con **Point-LIO/LIWO + relocalization + local map + planner predittivo**; e simulazione ad alta fedeltà dove il software vede quasi la stessa interfaccia che vede sul robot reale. citeturn22search2turn28search7turn17search19turn22search0turn20view2turn23search1

Il caso più istruttivo lato shooting è **Tongji SuperPower 2025**. Il loro open-source dichiara esplicitamente tre cose non banali: niente dipendenza ROS per il loop di auto-aim, una teoria di **“trajectory-view auto-aim”** con relativo planner, e un cambio del recognizer da pipeline tradizionale a detector neurale a quattro punti per migliorare il recall. Nel repo e nel relativo post riportano anche risultati quantitativi: errore stazionario inferiore a **0.01 rad** nel tracking di una singola armor a 5 rad/s e 3 m, hit-rate di gara fino a **39.6%** e normalmente non inferiore a **30%**, con tempi di kill di circa **8 s** a 7 rad/s e **10 s** a 14 rad/s contro un bersaglio da 300 HP a 2 m. Non è un A/B test pulito contro tutte le baseline, ma è abbastanza per dire che lì non stanno più ragionando come “PNP + EKF + soglia di fuoco”. citeturn6search4turn17search9turn14search1

Il caso più istruttivo lato navigation è **USTC RoboWalker 2025** e, in parallelo, **SMBU PolarBear 2025**. RoboWalker parla apertamente di uno stack sentry su ROS 2 basato su **Batch-LIWO, ROG-Map, MINCO-Omni-Planner e MPC Follower**, con funzionamento a **18 colpi/s** e capacità di lavorare contro target a **18 rad/s** con il **50% del fire window** e 20 ore di test di stabilità. PolarBear, invece, ha reso pubblico un pacchetto sim2real in cui la navigazione non è un modulo “isolato”, ma una piattaforma in cui virtual MCU, albero TF condiviso, relocalization `map -> odom`, simulatore Gazebo e tooling reale sono pensati insieme; in una issue pubblica del progetto spiegano anche di aver corretto problemi molto concreti della generazione precedente, come **TF tree non standard** e **timestamp non coerenti tra point cloud e odometry**. citeturn17search19turn22search0turn17search15turn21search13turn20view2

Il salto 2026 più visibile è invece **hardware-aware deployment**. Il post di ShanghaiTech sul progetto di auto-aim per **AX650** dichiara **300 fps di throughput** e **12 ms di latenza**, con codice, modelli e script di deployment completamente aperti; un altro repository del 2026, `BlueDarkUP/AIRS-RM-2025`, mostra una strada parallela su **Huawei Ascend Atlas 200I DK A2**, con detector YOLO, tracking Kalman e target scoring end-to-end. Il punto non è “comprare AX650 o Ascend a scatola chiusa”; il punto è che i team forti stanno trattando il deployment come parte dell’algoritmo, non come ultimo miglio. citeturn23search1turn35search1turn35search3turn35search0

## Cosa fanno di diverso oggi i team cinesi

La differenza più netta nello **shooting** è che i team cinesi più avanti **non decidono più il fuoco da un singolo errore istantaneo**. Tongji 2025 formalizza l’idea in termini di traiettoria: il problema non è “dove colpire adesso”, ma “quanto la traiettoria del gimbal e la traiettoria del bersaglio coincidono in un intervallo futuro”. COD 2026 porta questa logica su un dettaglio molto concreto e utilissimo: per i bersagli in rapido spin usa una `shooting_angle` **asimmetrica**, con finestra più larga sul lato “approaching” e più stretta sul lato “leaving” — nel post si parla di **55° contro 20°** — più un meccanismo di lock per evitare inseguimenti stupidi sull’armor che sta già uscendo dalla finestra utile. Questo è un cambio concettuale importante: il fire-control non è più un epilogo del tracker, ma un **problema di ottimizzazione vincolata nel tempo**. citeturn17search9turn14search8turn22search3turn23search3

Sul lato **CV / target state estimation**, il trend 2025–2026 è meno “magia AI” e più **modularità + stima fisica più coerente**. Tongji 2025 dichiara una struttura a moduli con recognizer, estimator, planner e controller; TARS 2026 dice esplicitamente di aver **abbandonato la soluzione tradizionale “single-frame PnP + post-filtering”**, usa **shared memory** per IPC ad alte prestazioni e punta a codice deployabile direttamente; il progetto `pb2025_rm_vision` espone in modo pragmatico tre backend di riconoscimento selezionabili a runtime — `opencv`, `openvino`, `tensorrt` — e un bringup ROS 2 già pronto. Questa combinazione fa capire che il livello cinese attuale è: tenere **una modellazione geometrica/temporale seria** e allo stesso tempo non vincolarsi a un solo runtime di inferenza. citeturn17search2turn22search2turn20view0

Sul lato **latency engineering**, uno dei riferimenti ancora molto forti è `julyfun/rm.cv.fans`. Il suo README non è recente, ma resta altamente rilevante perché i team 2025 lo citano ancora e perché incarna bene la scuola cinese: `lmtd_top_model.cpp`, `top_model.cpp`, `armor_model.cpp`, `coord_converter.cpp`, `aim_corrector.cpp`, `adaptive_ekf.hpp` e soprattutto `docs/auto_aim/latency.md`, che scompone la timeline in `img`, `predict`, `send`, `control`, `fire`, `hit`. Questa granularità è esattamente il contrario del classico “aggiungo 15 ms di offset e basta”; qui ogni tratto della catena viene misurato e compensato separatamente. citeturn28search0turn28search1turn28search7turn28search2

Nella **navigation**, i team cinesi recenti stanno facendo tre cose diverse dalla baseline occidentale/media ARC. Primo: stanno insistendo molto sul **sim2real vero**, non solo su “abbiamo Gazebo”. `rmu_gazebo_simulator` di PolarBear offre mondi RMUL/RMUC 2024–2025, controlli di chassis/gimbal/shooting e integrazione con il loro pacchetto di sentry navigation; lo stack `pb2025_sentry_nav` include componenti come `fake_vel_transform`, `point_lio`, `small_gicp_relocalization`, `terrain_analysis` e launch file dedicati per simulazione e realtà. Secondo: stanno dando importanza esplicita alla **relocalizzazione su mappa prior** e al mantenimento corretto di `map -> odom`, non solo all’odometria locale. Terzo: stanno passando a local planner più aggressivi e specifici, come **MINCO-Omni-Planner + MPC follower** nel caso RoboWalker. citeturn20view2turn22search0turn17search15turn17search19

C’è poi un filone 2024–2025 sul **radar/percezione off-board** che per ARCC 2026 considero secondario, ma che vale la pena notare come spia di maturità tecnica. HIT Shenzhen nel 2024 ha mostrato una pipeline radar station che dichiara **oltre 500 Hz** di processing e privilegia **point-cloud clustering + armor classification** invece di una pesante soluzione “dual neural network”; HKUST nel 2025 rivendica una soluzione radar monoculare con matching robusto per target veloci e occlusi, ispirata al filone PFA/Xiamen. Queste idee sono utili non tanto per copiare il radar station in ARCC, quanto perché mostrano che i team forti stanno **riducendo la dipendenza da detector monolitici** e stanno introducendo più spesso matching/stima geometrica ben progettati. citeturn8search4turn11search2turn17search4turn17search1

## Repository e file cinesi che prenderei subito in esame

Il repository che studiavo per primo è `TongjiSuperPower/sp_vision_25`. Anche quando il motore di ricerca non apre bene l’albero file completo, la pagina dell’organizzazione lo identifica chiaramente come repo principale 2025, e il post RM2025 dice esplicitamente che nel repo c’è il nuovo auto-aim planner; inoltre un post RM2026 di COD rimanda direttamente a **`aimer.cpp`** di `sp_vision_25` come riferimento per la logica di fuoco sui target rotanti. Se la vostra pipeline oggi usa ancora una decisione di fuoco euristica o simmetrica, questo è probabilmente il **punto singolo con ROI più alto** da studiare. citeturn19search7turn14search4turn22search3

Il secondo blocco è `julyfun/rm.cv.fans`, ma non per copiare tutto. I file che considero davvero utili da leggere sono: **`docs/auto_aim/latency.md`**, **`lmtd_top_model.cpp`**, **`top_model.cpp`**, **`coord_converter.cpp`**, **`aim_corrector.cpp`**, **`adaptive_ekf.hpp`**. In un singolo repo trovi, condensati, i tre temi che nei team cinesi tornano sempre: modellazione del top/spin, latenza trattata con disciplina, e correzione balistica chiusa sul dato reale. Questo repo non è “lo stato dell’arte 2026” in senso stretto, ma è ancora un riferimento tecnico importante e viene anche citato dall’ecosistema 2025. citeturn28search0turn28search1turn28search7turn28search2

Per la visione ROS 2 pronta all’uso prenderei `SMBU-PolarBear-Robotics-Team/pb2025_rm_vision`. I punti utili non sono solo il bringup `rm_vision_reality_launch.py`, ma anche **`dependencies.repos`**, che mostra in chiaro come compongono il workspace: `rmoss_interfaces`, `rmoss_core`, `hik_camera_ros2_driver`, `auto_aim_interfaces`, `pb_rm_interfaces`. Questo è prezioso perché non ti dà solo “un detector”, ti dà **la composizione software** che permette di far convivere visione, messaging e runtime multipli. citeturn20view0turn20view3turn22search2

Per la navigation prenderei quattro elementi del filone PolarBear: il repo `pb2025_sentry_nav`, il simulatore `rmu_gazebo_simulator`, i file di launch **`rm_navigation_simulation_launch.py`** e **`rm_navigation_reality_launch.py`**, e il modulo **`fake_vel_transform`**. Quest’ultimo è particolarmente interessante perché affronta un problema reale da ARCC/RM: il riferimento di velocità cambia significato quando il gimbal o il corpo stanno ruotando in modo non banale, e quindi la velocità “utile al tiro” e quella “utile alla navigation” possono divergere se non formalizzi bene i frame. Inoltre la issue pubblica sulla nuova versione segnala fix espliciti a **TF tree**, **timestamp pointcloud/odometry** e **sim performance**: esattamente i problemi che rovinano il sim2real nelle squadre meno mature. citeturn22search0turn20view2turn21search13turn22search4

Un quinto oggetto che guarderei è il filone **AX650 / accelerator-first deployment** di ShanghaiTech e, come alternativa hardware, `BlueDarkUP/AIRS-RM-2025` su Ascend. Non perché io consigli automaticamente di migrare a quei SoC, ma perché questi progetti mostrano **come si struttura un codice di auto-aim che nasce già con throughput, latenza e deploy script come requisiti di primo livello**. Se oggi la vostra pipeline vive bene solo in debug desktop e soffre appena la portate sul mini-PC di gara, è lì che va guardato il design pattern. citeturn23search1turn35search1turn35search0

## Implementazioni aggiuntive che integrerei oltre ai cinesi

La prima integrazione che farei, anche se non l’ho vista ancora standardizzata bene nei repo RM, è una **stima pose-weighted e uncertainty-aware** al posto del classico PnP con tutti i corner pesati uguali. In forma compatta, il problema corretto è

\[
\hat{\theta}
=
\arg\min_{\theta}
\sum_{i=1}^{N}
r_i(\theta)^\top \Sigma_i^{-1} r_i(\theta),
\qquad
r_i(\theta)=\pi(T(\theta)P_i)-u_i
\]

dove \(\Sigma_i\) è la covarianza 2D del corner o del keypoint, non un rumore isotropo fisso. La letteratura CVPR mostra che i solver PnP(L) uncertainty-aware migliorano la **mean translation accuracy del 18%** su un subset di KITTI, e che la refinement uncertainty-aware riduce l’errore medio di traduzione di **16%** per EPnP rispetto alla refinement standard sullo stesso dataset. In un contesto RoboMaster/ARCC non mi aspetto un +18% diretto in hit-rate, ma un miglioramento reale di **robustezza del pose stage** sì, soprattutto nei frame obliqui, parzialmente occlusi o con corner sbilanciati. citeturn26search1turn26search3turn26search14

La seconda integrazione che considero fortissima è passare dal filtro “one-step” classico a una **fixed-lag smoother** su una finestra corta con supporto naturale a misure asincrone e ritardate. La formulazione è

\[
\hat X_{k-L:k}
=
\arg\min_{X}
\sum_{t=k-L}^{k}
\left\|
r_{\text{imu}}^t
\right\|_{Q_t^{-1}}^2
+
\sum_j
\left\|
r_{\text{vision}}^j
\right\|_{R_j^{-1}}^2
+
\sum_m
\left\|
r_{\text{encoder}}^m
\right\|_{S_m^{-1}}^2
\]

e il vantaggio non è “solo più preciso”: è che timestamp diversi, ritardi di rete/seriale e time offset fra sensori non diventano più eccezioni da rattoppare, ma **vincoli naturali nel grafo**. La teoria e la pratica GTSAM vanno esattamente in questa direzione: smoothing incrementale, fixed-lag marginalization, e gestione naturale di sensori multi-rate e delayed. Per ARCC 2026 lo userei in due punti: **camera + IMU + gimbal encoder** per il targeting, e **LiDAR + IMU + wheel encoder** per la navigation. citeturn27search21turn27search0turn27search4turn27search17turn27search2turn36search14turn36search10

La terza integrazione è una gestione formalmente corretta delle **out-of-sequence measurements**. Qui i cinesi spesso ci arrivano per via empirica, misurando bene la latenza; io suggerisco di formalizzarlo. In tracking multisensore, una misura arrivata in ritardo ma aggiornata “come se fosse corrente” può peggiorare la stima anziché migliorarla. La letteratura OOSM e la documentazione Stone Soup lo mostrano chiaramente. In pratica questo significa: non fare update al volo quando arriva un packet seriale o un frame inferenza ritardato; invece, **mantieni una finestra temporale corta**, reinserisci la misura al suo timestamp di osservazione e riottimizza la coda della finestra. citeturn24search3turn24search22turn36search11turn36search20

La quarta integrazione, lato target tracking, è sostituire il singolo EKF con un **IMM a due o tre modelli** o, ancora meglio per RM/ARCC, con una doppia ipotesi esplicita **translation / low-spin / high-spin**. La formula di mixing di base è

\[
\mu_k^{(j)}
\propto
\Lambda_j(z_k)
\sum_i p_{ij}\,\mu_{k-1}^{(i)}
\]

dove \(\mu_k^{(j)}\) è la probabilità del modello \(j\), \(p_{ij}\) la probabilità di transizione e \(\Lambda_j\) la likelihood della misura. L’IMM è vecchio ma ancora estremamente competitivo nel rapporto costo/beneficio, ed è stato usato anche in letteratura di fire-control e target tracking manovrante come compromesso molto forte tra performance e complessità. Per ARMOR switching, tops e ingressi/uscite dalla finestra visiva, è più corretto di un EKF unico “elastico” caricato di tuning. citeturn25search14turn25search2turn31search3turn25search10

La quinta integrazione è sulla **navigation locale**. Se avete un robot omni o pseudo-omni e oggi usate un local planner reattivo più semplice, io guarderei seriamente due strade: **Nav2 MPPI** se volete qualcosa di industrializzabile in fretta, oppure **MINCO / GCOPTER-like optimization** se volete spingere di più su manovre aggressive e profili dinamici lisci. Nav2 documenta MPPI come controller predittivo successore di TEB/DWB, basato su sample-based optimization e critic functions estendibili; GCOPTER è un ottimizzatore di traiettorie costruito sopra MINCO e pensato proprio per trajectory optimization efficiente. Nel perimetro ARCC 2026, MPPI è la scelta più pragmatica; MINCO ha più upside, ma anche più costo d’integrazione. citeturn26search0turn26search8turn26search17turn24search20turn24search1

La sesta integrazione è di **dynamic-point / moving-event handling** nella percezione LiDAR. Se fate navigation/sentry in ambienti di gara con molti oggetti dinamici o disturbi dovuti ai colpi, un local map costruito assumendo che tutto sia statico è fragile. `M-detector` di HKU-MARS e il relativo paper su *Nature Communications* mostrano una detection point-by-point di moving events con latenza di **2–4 μs**, molto inferiore ai metodi frame-based. Non dico di trapiantarlo tale e quale nella vostra pipeline; dico che la direzione giusta è smettere di trattare il LiDAR come “frame cloud statica” quando la scena di gara non lo è. citeturn31search0turn31search1turn31search4

Per essere concreto, ecco due pezzi di codice che **integrerei davvero**.

Il primo è un solver C++ per **stima del tempo di impatto e dell’angolo di tiro** con latenza esplicita e drag quadratico. È una base molto più seria del classico “pitch correction per lookup table”, e si sposa bene con le idee di `latency.md` e dei planner di auto-aim cinesi. Le equazioni di fondo sono

\[
t_{\text{hit}}
=
t_{\text{img}}
+
\Delta_{\text{img}\to\text{predict}}
+
\Delta_{\text{predict}\to\text{send}}
+
\Delta_{\text{send}\to\text{control}}
+
\Delta_{\text{control}\to\text{fire}}
+
t_{\text{flight}}
\]

e

\[
m\dot{\mathbf v}
=
m\mathbf g
-
\frac{1}{2}\rho C_d A \|\mathbf v\|\mathbf v.
\]

Il codice seguente è originale, ma la logica è esattamente quella che oggi fa differenza in gara. citeturn28search7turn28search0turn17search9

```cpp
#include <cmath>
#include <array>
#include <stdexcept>

struct Vec3 {
  double x{}, y{}, z{};
  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
};

static inline double norm(const Vec3& v) {
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

struct DragConfig {
  double rho = 1.225;      // air density
  double cd  = 0.47;       // drag coefficient
  double area = 1.0e-4;    // projectile frontal area
  double mass = 0.0032;    // projectile mass
  double g = 9.81;
};

struct DelayChain {
  double img_to_predict = 0.0;
  double predict_to_send = 0.0;
  double send_to_control = 0.0;
  double control_to_fire = 0.0;

  double total() const {
    return img_to_predict + predict_to_send + send_to_control + control_to_fire;
  }
};

struct TargetKinematics {
  Vec3 pos_cam;  // target center in camera frame at t_img
  Vec3 vel_cam;  // target velocity in camera frame
};

Vec3 simulate_ballistic(double pitch, double yaw,
                        double speed,
                        double t_final,
                        const DragConfig& cfg,
                        double dt = 1e-3) {
  Vec3 p{0.0, 0.0, 0.0};
  Vec3 v{
      speed * std::cos(pitch) * std::cos(yaw),
      speed * std::cos(pitch) * std::sin(yaw),
      speed * std::sin(pitch)
  };

  const double k = 0.5 * cfg.rho * cfg.cd * cfg.area / cfg.mass;

  for (double t = 0.0; t < t_final; t += dt) {
    double s = norm(v);
    Vec3 a{
        -k * s * v.x,
        -k * s * v.y,
        -cfg.g - k * s * v.z
    };

    // RK2 / midpoint
    Vec3 v_mid = v + a * (0.5 * dt);
    double s_mid = norm(v_mid);
    Vec3 a_mid{
        -k * s_mid * v_mid.x,
        -k * s_mid * v_mid.y,
        -cfg.g - k * s_mid * v_mid.z
    };

    p = p + v_mid * dt;
    v = v + a_mid * dt;
  }

  return p;
}

// Solves for pitch such that projectile z matches future target z at range r_xy.
double solve_pitch_for_intercept(const TargetKinematics& target,
                                 double muzzle_speed,
                                 const DelayChain& delays,
                                 const DragConfig& cfg) {
  const double yaw = std::atan2(target.pos_cam.y, target.pos_cam.x);
  const double t0 = delays.total();

  // First-order target prediction to fire time.
  Vec3 target_fire = target.pos_cam + target.vel_cam * t0;
  const double r_xy = std::hypot(target_fire.x, target_fire.y);
  const double target_z = target_fire.z;

  auto residual = [&](double pitch) {
    // crude flight-time initialization by horizontal component
    double vx = muzzle_speed * std::cos(pitch);
    double t_flight = std::max(0.01, r_xy / std::max(1e-3, vx));
    Vec3 proj = simulate_ballistic(pitch, yaw, muzzle_speed, t_flight, cfg);
    return proj.z - target_z;
  };

  double lo = -0.30;   // about -17 deg
  double hi =  0.45;   // about  26 deg
  double r_lo = residual(lo);
  double r_hi = residual(hi);

  if (r_lo * r_hi > 0.0) {
    throw std::runtime_error("pitch intercept not bracketed");
  }

  for (int i = 0; i < 40; ++i) {
    double mid = 0.5 * (lo + hi);
    double r_mid = residual(mid);
    if (std::abs(r_mid) < 1e-4) return mid;
    if (r_lo * r_mid <= 0.0) {
      hi = mid;
      r_hi = r_mid;
    } else {
      lo = mid;
      r_lo = r_mid;
    }
  }
  return 0.5 * (lo + hi);
}
```

Il secondo pezzo è un **fire gate spin-aware** con finestra asimmetrica e previsione della fase dell’armor. È esattamente la categoria di logica che nei repo cinesi recenti sta sostituendo le threshold simmetriche troppo rozze. citeturn23search3turn17search9turn28search0

```cpp
#include <cmath>
#include <algorithm>

struct SpinState {
  double cx, cy, cz;     // target center
  double vx, vy, vz;     // linear velocity
  double phase;          // current armor phase [rad]
  double omega;          // angular speed [rad/s]
  double radius;         // armor radius from center [m]
};

struct FireWindow {
  double approach_deg = 55.0; // generous when armor is becoming favorable
  double leave_deg    = 20.0; // strict when armor is leaving
  double max_yaw_err  = 0.010; // rad
  double max_pitch_err = 0.012; // rad
};

static inline double wrap_pi(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// alpha_i is armor index phase offset, e.g. 0, pi/2, pi, 3pi/2 for 4-armor model.
double armor_phase_at_fire(const SpinState& s, double dt_fire, double alpha_i) {
  return wrap_pi(s.phase + s.omega * dt_fire + alpha_i);
}

bool should_fire_spin_target(const SpinState& s,
                             double dt_fire,
                             double alpha_i,
                             double yaw_err,
                             double pitch_err,
                             const FireWindow& cfg) {
  double phase = armor_phase_at_fire(s, dt_fire, alpha_i);

  // approaching if derivative moves armor toward frontal visibility around 0 rad
  // sign convention can be flipped depending on your frame definition
  bool approaching = (s.omega * phase < 0.0);

  double gate_deg = approaching ? cfg.approach_deg : cfg.leave_deg;
  double phase_ok = std::abs(phase) < gate_deg * M_PI / 180.0;
  double aim_ok   = std::abs(yaw_err) < cfg.max_yaw_err &&
                    std::abs(pitch_err) < cfg.max_pitch_err;

  return phase_ok && aim_ok;
}
```

## Valutazione quantitativa e priorità di integrazione

Le percentuali qui sotto sono **stime ingegneristiche mie**, non claim ufficiali dei team; dove esistono numeri di paper o di team, li cito e li distinguo chiaramente. La logica è: quanto mi aspetto che cambi davvero la vostra pipeline ARCC 2026, se oggi siete ancora su una base “detector + PnP + EKF + gate di fuoco + nav standard”.

| Intervento | Effetto atteso | Quanto credo valga davvero |
|---|---:|---|
| **Latency chain esplicita + fire gate asimmetrico** | **+4 a +10 punti** di hit-rate sui target in spin medio/alto | Alto, perché è già la direzione convergente di Tongji, COD e `rm.cv.fans` |
| **Trajectory-view auto-aim planner** | **+3 a +8 punti** di hit-rate e minore tempo di kill | Medio-alto, se il vostro collo di bottiglia è la decisione di fuoco e non il detector |
| **PnP uncertainty-aware** | **-10% a -18%** sull’errore di posa al livello solver; **+1 a +4 punti** di hit-rate attesi in gara | Medio, ma robusto dal lato matematico |
| **Fixed-lag smoothing con OOSM/time-offset handling** | **+2 a +6 punti** di hit-rate e forte riduzione degli spike di errore | Medio-alto se soffrite di latenze asincrone reali |
| **IMM / multi-hypothesis translation-spin tracker** | **+2 a +7 punti** di hit-rate sui cambi regime del bersaglio | Medio |
| **Point-LIO/LIWO + relocalization + local map seria** | Nav più stabile; mi aspetto **-30% a -60%** dei failure di relocalization/planner rispetto a stack improvvisati | Alto per sentry/engineer, basso per robot quasi-statici |
| **MPPI o planner MINCO-like** | **-15% a -30%** in overshoot/oscillazione locale, con traiettorie più pulite | Medio |
| **Deployment accelerator-first** | Non aumenta la precisione “per magia”, ma può togliere **10–30 ms** effettivi di budget | Alto se oggi siete compute-bound |

I tre interventi che farei **subito** sono i primi tre della tabella. La ragione è semplice. La pratica cinese recente mostra chiaramente che il ROI massimo non sta nel cambiare il detector ogni mese, ma nel trattare **tempo, traiettoria e incertezza** come oggetti di primo livello. I numeri di Tongji 2025, la decomposizione delle latenze in `rm.cv.fans`, l’asimmetria esplicita di COD 2026 e le pipeline 300 fps / 12 ms di ShanghaiTech convergono tutti sulla stessa tesi: **quando la catena tempo-modello-controllo è coerente, il resto pesa molto di più**. citeturn6search4turn17search9turn28search7turn23search3turn23search1

Sul lato navigation, se ARCC 2026 per voi significa davvero 3v3 + engineer e non state costruendo un sentry complesso, allora sarei pragmatico: **non porterei in blocco tutta la complessità RoboWalker/PolarBear**. Porterei invece soltanto ciò che ha alta resa e basso rischio: **timestamp correctness, TF correctness, relocalization robusta, simulator serio, e un local planner predittivo maturo**. Il resto — ROG-Map custom, MINCO omni aggressivo, pipeline M-detector, BA full-stack LIWO — lo metterei in seconda ondata. L’errore tipico dei team occidentali non è “usare algoritmi troppo semplici”; è **integrare troppa roba avanzata senza chiudere bene timing e frame semantics**. citeturn21search13turn17search15turn20view2turn26search0turn24search2turn32search1

In sintesi, il mio verdetto è questo: **la differenza cinese 2024–2026 non è principalmente nell’AI model, ma nell’ingegneria matematica della pipeline**. Per ARCC 2026, se dovessi scegliere una traiettoria tecnica seria e realistica, andrei così: **latency-aware ballistic fire control**, poi **trajectory/spin-aware target estimation**, poi **uncertainty-aware pose + fixed-lag smoothing**, e solo dopo **navigation stack avanzato** e **accelerator-specific deployment**. È la combinazione che, con il miglior rapporto costo/beneficio, ha più probabilità di aumentare insieme **accuratezza**, **precisione** e **ripetibilità reale in gara**. citeturn30search1turn30search3turn17search9turn17search19turn23search1turn26search1turn27search2