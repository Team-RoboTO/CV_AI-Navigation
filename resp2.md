Diagnosi: il problema NON è il servo, è la stima
Dato chiave da log2.txt: yaw_error ≈ 0 in tutti i frame → il gimbal segue cmd_yaw benissimo. Quindi l'overshoot/drift non è ritardo del servo: è cmd_yaw stesso ad essere una stima che lagga e poi sbanda. Due cause, entrambe nel codice:

Causa A (dominante) — sovra-smoothing isotropo → lag + overshoot allo stop.
measurementNoise modellava il rumore di posizione uguale su tutti gli assi: a 2.5 m R = (0.05 + 0.04·2.5)² = 0.15² m². Ma il PnP monoculare è preciso in bearing (~1–2 cm laterali) e impreciso solo in profondità. Costringere il canale laterale (preciso) a portare 15 cm di rumore fa sì che l'EKF sovra-smoothi → la velocità stimata lagga in movimento e persiste quando il nemico si ferma (ci mette molti frame ad accettare v→0) → la mira continua ad anticipare un bersaglio ormai fermo = overshoot. Prova nel log: pd (innovazione di posizione) è 6–11 cm e viene "accettato" come rumore invece di correggere la traccia.

Causa B — vyaw fantasma dall'accoppiamento posizione↔yaw nell'EKF.
Lo Jacobiano accoppia posizione e spin (H(0,6)=r·sinθ, H(1,6)=−r·cosθ). Su un bersaglio che trasla (non gira), l'errore di lead in posizione viene in parte attribuito allo yaw → vyaw ≈ −0.10 fantasma. Poi computeAim applicava un lead rotazionale (θ_impatto = yaw + vyaw·T) → un drift laterale costante (~4 mm qui, ma cresce con velocità/vicinanza). Aggravante: det_yaw oscilla ±14° con reproj 0.01px (yaw non osservabile quasi-frontalmente) — quindi quel vyaw è puro rumore.

Fix applicate (build OK, 1min16s)
Fix	File	Cosa fa
Rumore di misura anisotropo	tracker.cpp::measurementNoise	Separa radiale (profondità, grande, r_pos_*) da tangenziale (laterale, piccolo, nuovi r_pos_tang_base=0.025, r_pos_tang_slope=0.010 → ~0.05 m a 2.5 m invece di 0.15). Blocco 2×2 = diag(rad,tang) ruotato sul bearing (PD garantita). L'EKF segue il laterale ~3× più stretto → tracking migliore e stop più rapido → meno overshoot.
Gating del lead rotazionale	tracker.cpp::computeAim	Il lead da vyaw si applica solo a spin confermato dal timing (phase_timing_confident_); altrimenti la mira usa vyaw=0. Il lead di traslazione resta sempre attivo. Elimina il drift fantasma sui bersagli che traslano.
Tutto tunabile e reversibile (metti r_pos_tang_* == r_pos_* per tornare all'isotropo). Documentato in WORKLOG §4b.

Nota di onestà: il log è una finestra di tracking a regime (non cattura l'istante di stop); l'overshoot è dedotto meccanicisticamente dalle prove di sovra-smoothing (pd grande accettato) e dal vyaw fantasma, che producono esattamente quel comportamento.

Da validare/tarare sul robot (WORKLOG §4b.5)
Nemico che strafe: cmd_yaw deve anticipare liscio e assestarsi senza overshoot quando si ferma; pd deve calare.
Se il laterale è jittery → alza r_pos_tang_base verso 0.04–0.05. Se ancora lagga → abbassa r_pos_tang_slope verso 0.005.
Se il lead a regime trascina → prova alpha_pos = 1.0 (CV puro) — l'ho lasciato come leva documentata, non l'ho toccato per non muovere troppe variabili insieme.
Inconsistenze rimaste (documentate, non toccate)
alpha_pos=0.995 introduce un bias che sotto-anticipa in moto costante (è in conflitto con il volere "seguire bene"); q_pos=10 è reattivo. Sono i prossimi due lever se serve.
Il salto strutturale vero per accuratezza+bullet economy resta il gate su probabilità di colpire (roadmap #1 di ricerca1.md) e la ego-motion per lo shoot-on-move.
Vuoi che proceda con alpha_pos = 1.0 (test rapido del lead a regime) o preferisci prima validare queste due fix sul campo?