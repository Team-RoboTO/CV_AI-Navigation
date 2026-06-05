# !!!! AI WARNING !!!!
# Auto-Aim - Revisione codice e proposte di miglioramento

Branch: `autoaim/shooting_tracker` · Data: 2026-06-01 · Revisione originale: Claude (su richiesta) · Riscrittura: Codex

Questo documento è una lista di **proposte tecniche**. Non implica che il codice
sia stato modificato. I punti sono ordinati per priorità e distinguono tra bug
probabili, fragilità, pulizia del repository, miglioramenti matematici e test.

## Legenda

- 🔴 **Bug / correttezza**: può causare comportamento sbagliato in gara.
- 🟡 **Rischio / fragilità**: oggi può funzionare, ma è facile da rompere,
  fraintendere o tarare male.
- 🟢 **Pulizia / manutenibilità**: non è urgente, ma migliora qualità e
  leggibilità del repository.
- 📐 **Matematica**: modello, stima, balistica, filtro.
- 🧪 **Test / processo**: verifiche automatiche, CI, regressioni.

## Valutazione generale

L'overhaul è una **buona direzione**: consolidare i 4 pacchetti in
`src/autoaim/`, usare un nodo composable, passare a un EKF "spinning top" 9D e
integrare un detector YOLO26-pose rende il sistema più leggibile e più vicino a
un'architettura collaudata in stile `rm_vision`.

La matematica di base è sana: il tracker usa uno stato coerente, la forma di
Joseph per aggiornare `P`, e LDLT per il gain. I punti sotto sono quindi
**rifiniture e riduzione del rischio**, non una bocciatura del lavoro fatto.

Le priorità pratiche sono:

1. correggere subito le incongruenze che possono cambiare il comportamento in
   campo (`class_id`, gating/update, flip camera, IMU finta);
2. rendere più chiara e meno fragile la calibrazione del gimbal;
3. spostare configurazioni e documentazione in file dedicati;
4. aggiungere test automatici sulla matematica del tracker.

---

## 1. 🔴 Bug e problemi di correttezza

### 1.1 Convenzione `class_id`: allineata a YOLO26

**Situazione attuale**

- Il detector nuovo, in `zed_detector.py`, usa:
  `CLASS_NAMES = {0: blue, 1: grey, 2: red}`.
- Quindi `class_id` può essere solo `"0"`, `"1"` o `"2"`.
- In `autoaim_node.cpp` il default è ora:
  `target_classes = {"0"}`.
- I commenti runtime, `README.md` e `media/docs/field_manual.md` sono allineati:
  `"0" = blue`, `"1" = grey`, `"2" = red`.

**Perché conta**

Con il modello attuale il rosso è `"2"`, non `"3"`. I launch
`standard.launch.py`, `hero.launch.py` e `sentry.launch.py` puntano ancora a
`["0"]`, quindi attaccano il blu. Per attaccare il rosso bisogna usare `["2"]`.

- `target_classes_.erase("1")` rimuove il grigio assumendo che `"1" = grey`.
  Questo è vero con il modello nuovo ed è documentato come contratto.

**Resta da fare**

Idealmente centralizzare la mappa classi in un unico punto condiviso tra
detector e nodo, così non può divergere di nuovo.

### 1.2 Gating Mahalanobis non coerente con l'update

**Situazione attuale**

- `ekfUpdate()` costruisce una matrice `R` che viene scalata anche in base
  all'obliquità del target (`xyz_f`, `yaw_f`, taglio a `max_oblique_deg`).
- `ekfMahalanobis()`, usato per associazione e gating, costruisce invece una
  `R` dipendente solo dalla distanza.

**Perché conta**

Il gating decide se una detection è compatibile usando una covarianza diversa
da quella usata subito dopo per correggere lo stato. A grandi angoli questo può
produrre decisioni incoerenti:

- detection buone possono essere scartate;
- detection cattive possono essere accettate;
- il filtro può diventare più nervoso proprio nelle condizioni geometriche più
  difficili.

**Proposta**

Usare la stessa costruzione di `R` in entrambe le funzioni. La soluzione più
semplice è estrarre una funzione condivisa, ad esempio `buildR(z)`, che includa
sia la dipendenza dalla distanza sia quella dall'obliquità.

### 1.3 `dist_now` in `computeAim` ignora l'altezza del gimbal

**Situazione attuale**

In `tracker.cpp:699` viene calcolato:

```cpp
dist_now = sqrt(dx² + dy² + x_(4)²)
```

Qui `x_(4)` rappresenta la quota `z` del centro target sopra il suolo. La canna,
però, non parte dal suolo: parte da `gimbal_height`.

**Perché conta**

`targetRange()` sottrae correttamente `gimbal_height`, mentre `computeAim()` no.
Questa incoerenza interna produce una piccola sovrastima della distanza e quindi
un piccolo errore sul tempo di volo, nell'ordine di qualche millisecondo. Non
sembra critico, ma è un errore facile da eliminare.

**Proposta**

Calcolare la componente verticale rispetto alla canna:

```cpp
x_(4) - gimbal_height
```

invece di usare direttamente `x_(4)`.

### 1.4 IMU del detector: codice morto che sembra vivo

**Situazione attuale**

In `zed_detector.py:644-645` la chiamata:

```python
self.zed.get_sensors_data(...)
```

è commentata. Di conseguenza:

- `sensors_data` resta un oggetto vuoto;
- `get_imu_data()` restituisce valori nulli o non validi;
- i controlli di validità sulla norma del quaternione sono commentati;
- `/zed/imu_data` pubblica un orientamento spazzatura o identità.

**Perché conta**

L'impatto reale sembra basso, perché a valle l'orientamento usato dal sistema
arriva da `/micro_status`, non dalla ZED. Però il publisher `/zed/imu_data`
sembra vivo e affidabile, mentre in realtà non lo è. Questo può confondere molto
durante debug e integrazione.

**Proposta**

Scegliere una delle due strade:

- ripristinare davvero la lettura IMU dalla ZED e riattivare i controlli di
  validità;
- rimuovere del tutto il publisher `/zed/imu_data` e la funzione `_publish_imu()`
  se quel dato non viene usato.

### 1.5 Commento contraddittorio su `ego_position_max_drift`

**Situazione attuale**

In `autoaim_node.cpp` il default in codice è:

```cpp
ego_position_max_drift = 4.0
```

Poco sotto, però, il commento dice che il default `0.0` disabilita interamente
la funzione. Il launch passa effettivamente `0.0`.

**Perché conta**

Il comportamento del codice è comprensibile, ma il commento no. Chi deve tarare
il parametro rischia di non capire se il default reale sia `4.0` o `0.0`, e se
la feature sia attiva oppure disattivata.

**Proposta**

Correggere il commento e rendere coerenti:

- default dichiarato nel codice;
- default usato nel launch;
- descrizione del comportamento quando il valore è `0.0`.

### 1.6 `camera_image_flip = ON` forzato

**Situazione attuale**

In `zed_detector.py:254` c'è:

```python
params.camera_image_flip = sl.FLIP_MODE.ON
```

Il commento vicino dice invece di abilitarlo solo se la camera è fisicamente
montata capovolta.

**Perché conta**

Se la ZED non è montata capovolta, questo ribalta l'immagine e può compromettere
tutta la pipeline:

- keypoint;
- PnP;
- odometria;
- associazione con il tracker.

**Proposta**

Verificare fisicamente il montaggio della camera. Poi rendere il flip un
parametro di launch, non una costante hard-coded. Il commento deve descrivere il
comportamento reale del codice.

---

## 2. 🟡 Fragilità e rischi di calibrazione

### 2.1 Gestione dei segni di pitch troppo intricata

**Situazione attuale**

Ci sono tre flag di segno e un offset che influenzano il pitch:

- `gimbal.pitch_sign`;
- `micro_pitch_feedback_opposite_sign`;
- `micro_pitch_lock_opposite_sign`;
- `pitch_offset_deg`.

Questi parametri agiscono in punti diversi: geometria PnP, fire-lock e comando
finale.

**Perché conta**

Questa è la zona a più alto rischio di errore del tipo: "la canna punta in
basso, ma il codice crede che punti in alto". Quando i segni sono distribuiti in
più punti, una taratura può compensare accidentalmente un errore precedente e
rendere il sistema difficile da capire.

**Proposta**

Definire una procedura di calibrazione unica e documentata. Ridurre il segno del
pitch a un solo punto di verità, oppure spiegare chiaramente quale parametro
corregge quale convenzione. Aggiungere anche un controllo di coerenza all'avvio,
ad esempio: inclinazione nota del gimbal → segno atteso nel feedback.

### 2.2 Logica `dz_` fragile nella gestione dei salti tra piastre

**Situazione attuale**

`handleArmorJump()` ricostruisce segno e magnitudine di `dz_` usando:

- EMA sul valore assoluto;
- ripristino del segno;
- protezioni multiple;
- correzioni per evitare divergenze già citate nei commenti.

**Perché conta**

La logica è difficile da seguire e delicata. Inoltre la geometria del robot non
cambia frame per frame: stimarla continuamente con una EMA rumorosa introduce
complessità e casi limite.

**Proposta**

Modellare le 4 piastre in modo deterministico:

- due raggi `r0`, `r1`;
- due quote `z0`, `z1`;
- valori misurati una volta e calibrati per robot.

Lo spinning-top RoboMaster ha una geometria fissa nota. Per questo è meglio
usare costanti calibrate invece di stimare `dz_` frame-by-frame.

### 2.3 Parametri dei profili quasi identici

**Situazione attuale**

`standard.launch.py`, `hero.launch.py` e `sentry.launch.py` differiscono
principalmente per il path del modello. Parametri come questi risultano
identici:

- `gimbal_height = 0.420`;
- offset canna;
- segni del gimbal;
- parametri principali del tracker.

**Perché conta**

Se i due robot fisici sono diversi, questi valori devono essere calibrati per
robot. Tenerli duplicati nei launch rende facile dimenticare una differenza
meccanica reale.

**Proposta**

Spostare i parametri in file YAML separati, ad esempio:

- `config/standard.yaml`;
- `config/hero.yaml`;
- `config/sentry.yaml`.

In questo modo le differenze per robot diventano esplicite, versionabili e più
facili da revisionare.

### 2.4 `/camera_info` ricalcolato e pubblicato a 120 Hz

**Situazione attuale**

`_publish_camera_info()` chiama `get_camera_information()` a ogni frame e
pubblica un messaggio `CameraInfo` completo a 120 fps.

**Perché conta**

Gli intrinsics della camera sono costanti durante il runtime. Ricalcolarli e
ripubblicarli a ogni frame spreca CPU e banda senza aggiungere informazione.

**Proposta**

Calcolare `CameraInfo` una sola volta. Poi pubblicarlo:

- con QoS `transient_local`, quindi in modo latched per i subscriber tardivi; o
- a bassa frequenza, ad esempio 1-2 Hz.

### 2.5 Distorsione assunta nulla

**Situazione attuale**

`zed_detector.py` pubblica:

```python
d = [0, 0, 0, 0]
```

**Perché conta**

Questa assunzione va bene solo se l'immagine `LEFT` della ZED è già rettificata,
cosa che di solito è vera. Se invece l'immagine non è rettificata, il PnP avrà
un errore sistematico, soprattutto ai bordi.

**Proposta**

Confermare esplicitamente che il frame usato sia rettificato. Poi documentare
l'assunzione nel codice o nella documentazione di architettura.

---

## 3. 🟢 Pulizia e struttura del repository

### 3.1 533 MB di engine, ma uno solo è usato

**Situazione attuale**

`models/{jetson16/standard,jetson16/hero,jetson64}/` contiene o conterra' gli
engine `.engine` per profilo/piattaforma:

- `rtdetr`;
- `yolov8*`;
- `yolov11*`;
- `yolo26_bbox`;
- `yolov26_keypoints`.

Il codice usa solo:

```text
yolov26_keypoints.engine
```

In totale sono circa 533 MB su disco. È corretto che non siano tracciati da git,
ma restano peso e ambiguità nel workspace.

**Perché conta**

Gli engine TensorRT sono legati a GPU e versione TRT. Non sono portabili come
artefatti generici. Tenere engine non usati aumenta il rischio di lanciare il
modello sbagliato.

**Proposta**

Tenere solo `yolov26_keypoints.engine` per piattaforma. Archiviare altrove:

- i sorgenti `.onnx` o `.pt`;
- uno script per rigenerare gli engine;
- eventuali modelli sperimentali non usati in gara.

Promemoria importante: i modelli per la **Jetson 16 sono placeholder** e non
ancora allenati. Va scritto chiaramente nel README, così nessuno li usa in gara
per errore.

### 3.2 Documentazione di architettura mancante

**Situazione attuale**

L'overhaul ha rimosso la vecchia documentazione in `media/docs/`, inclusi file
come:

- pipeline;
- guida di calibrazione;
- contratto `gimbal_command_contract`;
- altri appunti utili per debug e integrazione.

**Perché conta**

L'obiettivo dichiarato è aiutare a debuggare e capire il sistema. Senza una
documentazione minima, i dettagli critici restano dispersi nel codice e nei
launch.

**Proposta**

Creare un singolo documento conciso, ad esempio `docs/ARCHITECTURE.md`, che
contenga almeno:

1. schema di topic e frame, cioè chi pubblica cosa;
2. guida di calibrazione per segni gimbal, offset canna e `bullet_speed`;
3. contratto del comando `/cmd_vel_AI`, inclusi i campi usati del `Twist`.

Non serve ripristinare i 15 file precedenti. Basta un documento breve ma
affidabile.

### 3.3 Parametri inline nel launch invece che YAML

**Situazione attuale**

I circa 60 parametri principali sono dizionari Python dentro i launch. Nel codice
vecchio esistevano file `params_*.yaml`.

**Perché conta**

I file YAML rendono più semplici:

- diff e review;
- separazione tra robot;
- override dei parametri;
- tuning e reconfigure a runtime, quando il nodo lo supporta;
- riuso della stessa configurazione in launch diversi.

**Proposta**

Spostare i parametri in:

- `config/standard.yaml`;
- `config/hero.yaml`;
- `config/sentry.yaml`.

I launch dovrebbero limitarsi a scegliere quale configurazione caricare e quali
override applicare.

### 3.4 Contratto del `Twist` `/cmd_vel_AI` implicito

**Situazione attuale**

Il comando `/cmd_vel_AI` usa `geometry_msgs/Twist` per trasportare campi con
significati non ovvi:

- `angular.x` = fire;
- `angular.y` = pitch;
- `angular.z` = yaw;
- `linear.x` = distanza.

**Perché conta**

Funziona, ma è un uso improprio o comunque non autoesplicativo del messaggio
`Twist`. Chi legge il topic senza conoscere il contratto può interpretarlo male.

**Proposta**

Documentare esplicitamente il contratto. Inoltre esiste già `GimbalCmd.msg` nel
pacchetto: vale la pena valutare se usarlo al posto del `Twist`, così il tipo del
messaggio descrive direttamente i campi reali.

---

## 4. 📐 Matematica: valutazione e alternative

### Modello attuale

Il modello attuale è corretto e collaudato: EKF 9D con stato

```text
[xc, vxc, yc, vyc, za, vza, yaw, vyaw, r]
```

La predict usa velocità costante con damping:

```text
alpha^(dt · ref_freq)
```

La `Q` deriva da accelerazione bianca. L'update usa la misura:

```text
[xa, ya, za, yaw]
```

con `R` dipendente da distanza e obliquità. Il tracker gestisce inoltre:

- jump di faccia a circa 90°;
- raggio tramite EMA;
- stima di `vyaw` dal timing dei jump, usando `π/2` per salto;
- forma di Joseph per `P`;
- LDLT per il gain.

Queste sono scelte numericamente sane.

### 4.1 Damping + `Q`: possibile doppio conteggio

**Situazione attuale**

Il filtro usa sia:

- damping moltiplicativo sulle velocità, cioè `v *= alpha`;
- `Q` da white-noise-acceleration.

**Perché conta**

Queste due scelte mescolano due filosofie. Per un target che manovra, un modello
a **accelerazione costante (CA)** o un **IMM** (Interacting Multiple Model, ad
esempio CV + CT) gestisce meglio start, stop e cambi di spin rispetto a un
damping euristico.

Il damping è utile quando il target è in coast o temporaneamente perso, ma in
tracking nominale introduce un bias sistematico che rallenta la stima della
velocità.

**Proposta a basso rischio**

Mantenere il damping solo in `TEMP_LOST`, cioè durante il coast. In tracking
nominale impostare `alpha_pos` vicino a `1.0` o uguale a `1.0`, lasciando che sia
`Q` a regolare la reattività del filtro.

### 4.2 `vyaw` dal timing: meglio fonderlo che sovrascriverlo

**Situazione attuale**

La stima:

```text
vyaw = (π/2) / dt
```

è molto utile, perché converge in circa 2 salti invece che in 10-20 frame. Oggi
però viene blendata a mano, prima con peso `0.8` e poi `1.0`, e si forza
direttamente `P(7,7)`.

**Perché conta**

La stima è buona, ma il modo in cui viene integrata nel filtro è meno pulito del
resto dell'EKF. Le costanti manuali rendono più difficile capire quanto il
filtro si fidi davvero della misura di `vyaw`.

**Proposta**

Trattare `vyaw` da timing come una misura scalare con una propria varianza,
derivata dall'incertezza su `dt`. Poi fonderla con un update EKF scalare. Si
ottiene lo stesso beneficio, ma in modo coerente con il resto del filtro.

### 4.3 Geometria delle 4 piastre come stato/calibrazione, non EMA

Questo è lo stesso punto di 2.2 visto dal lato matematico. I due raggi e le due
quote delle piastre sono fissi e noti per ogni robot RoboMaster:

- `r0`, `r1`;
- `z0`, `z1`.

Stimarli con EMA introduce rumore e crea i casi limite di `dz_`. Meglio misurarli
una volta e trattarli come costanti calibrate, con al massimo una piccola
correzione opzionale del raggio principale.

### 4.4 Balistica: modello flat-fire senza drag

**Situazione attuale**

`solveBallistic()` itera una formula del tipo:

```text
pitch = atan2(dz + 1/2 · g · t², gd)
```

Questo è corretto per una traiettoria con gravità, ma trascura la resistenza
dell'aria.

**Perché conta**

A `25 m/s` e distanze fino a circa `6 m`, l'errore dovuto al drag è piccolo ma
non nullo, soprattutto con proiettili leggeri da 17 mm o 42 mm. Inoltre
`bullet_speed` è hard-coded a `25 m/s`, mentre andrebbe misurato sul robot,
come già indicato nei commenti.

**Proposte**

Due strade possibili:

- aggiungere un termine di drag lineare, ad esempio `v̇ = -k · v`;
- usare una tabella di calibrazione misurata, cioè compensazione pitch vs
  distanza.

Nel vecchio codice esisteva un `calibration_table_template.csv`: l'idea è valida
e può essere recuperata. Ancora meglio, il microcontrollore potrebbe inviare la
`bullet_speed` reale in `/micro_status`, così il valore si aggiorna a runtime.

### 4.5 Approccio alternativo più ambizioso

Alcune squadre tracciano simultaneamente tutte le piastre visibili dentro un
unico EKF del robot intero. In quel caso ogni frame può contenere più
osservazioni, non solo una piastra scelta.

**Vantaggio**

La stima dello spin diventa più robusta e dipende meno dal singolo jump.

**Nota pratica**

L'approccio attuale, cioè singola piastra più jump, è più semplice e già
collaudato. Conviene considerare il modello multi-piastra solo se il jitter di
`vyaw` ad alti RPM resta un problema dopo le correzioni più semplici.

---

## 5. 🧪 Test e processo

### 5.1 Nessun test automatico

**Situazione attuale**

Non ci sono test automatici. Dato che l'obiettivo è debuggare meglio, questa è
probabilmente la cosa con il ritorno più alto.

**Perché conta**

La matematica del tracker è C++ puro e si presta bene a test deterministici.
Senza test, ogni taratura o refactor rischia di rompere casi già risolti.

**Proposta**

Aggiungere un target `ament_add_gtest` con almeno questi casi:

- `ekfPredict` / `ekfUpdate` su traiettoria sintetica nota, verificando la
  convergenza;
- `handleArmorJump` con jump a 90° e 180°, verificando swap del raggio e segno di
  `dz`;
- `solveBallistic` contro soluzioni analitiche a varie distanze;
- `shouldSwitch`, includendo la regressione "stuck in DETECTING" citata nei
  commenti.

### 5.2 CI minima

**Situazione attuale**

Non è indicato un controllo automatico di build e test.

**Proposta**

Aggiungere un workflow minimo che esegua:

```bash
colcon build
colcon test
```

oppure `colcon build` più i gtest del tracker. Anche una CI semplice evita molte
regressioni durante debug e modifiche rapide in periodo gara.

---

## 6. Riepilogo operativo per priorità

| Priorità | Tipo | Voce | Sforzo |
|---|---|---|---|
| 1 | ✅ | Convenzione classi `0/1/2` allineata tra codice e docs | fatto |
| 2 | 🔴 | IMU detector commentato: `/zed/imu_data` pubblica dati non affidabili | basso |
| 3 | 🔴 | `camera_image_flip = ON` forzato: verificare montaggio e parametrizzare | basso |
| 4 | 🔴 | `R` di gating diversa dalla `R` di update: includere obliquità in entrambe | basso |
| 5 | 🔴 | `dist_now` non sottrae `gimbal_height` | basso |
| 6 | 🔴 | Commento incoerente su `ego_position_max_drift` | basso |
| 7 | 🟡 | Tre flag di segno pitch più offset: unificare o documentare calibrazione | medio |
| 8 | 🟡📐 | `dz_`, raggi e quote: usare costanti calibrate invece di EMA rumorosa | medio |
| 9 | 🧪 | Aggiungere gtest sul tracker | medio |
| 10 | 📐 | Damping vs `Q`: limitare damping al `TEMP_LOST` | medio |
| 11 | 📐 | `vyaw` dal timing: trattarlo come misura EKF scalare | medio |
| 12 | 📐 | Balistica: drag o tabella pitch-vs-distanza, più `bullet_speed` misurata | medio |
| 13 | 🟡 | Parametri dei profili separati per robot | medio |
| 14 | 🟡 | `/camera_info` pubblicato a 120 Hz: calcolare una volta o pubblicare lento | basso |
| 15 | 🟡 | Distorsione nulla: documentare che l'immagine ZED è rettificata | basso |
| 16 | 🟢 | Rimuovere engine inutilizzati e segnare Jetson 16 come placeholder | basso |
| 17 | 🟢 | Ripristinare una documentazione minima di architettura | medio |
| 18 | 🟢 | Spostare i parametri dai launch a YAML | medio |
| 19 | 🟢 | Documentare o sostituire il contratto implicito del `Twist` `/cmd_vel_AI` | basso |
| 20 | 🧪 | Aggiungere CI minima con build e test | medio |

## 7. Sequenza consigliata

Per ridurre il rischio senza aprire troppi fronti insieme:

1. correggere `camera_image_flip`, IMU e `dist_now`;
2. aggiungere una piccola documentazione di calibrazione per pitch e classi;
3. introdurre YAML per `standard`, `hero` e `sentry`;
4. aggiungere i primi gtest su balistica, predict/update e armor jump;
5. solo dopo intervenire sulle parti più matematiche, come damping, `vyaw` da
   misura e geometria fissa delle piastre.
